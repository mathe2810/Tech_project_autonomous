#!/usr/bin/env python3
"""
Keyboard teleop - control robot with WASD keys.
+ Minimal AUTO mode (naive wall-follow two-beam + front safety) using /scan.

Added keys:
  M : Toggle AUTO/MANUAL
  R : AUTO follow RIGHT wall
  L : AUTO follow LEFT wall
  SPACE : Stop (also disables AUTO)
"""

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan  # ADDED
from rclpy.qos import qos_profile_sensor_data  # ADDED
import sys
import termios
import tty
import select  # ADDED
import math    # ADDED
import time    # ADDED


def get_key():
    """Get a single key press"""
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        return ch
    finally:
        # NOTE: This matches your original style (no refactor here).
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))


def main():
    rclpy.init()
    node = rclpy.create_node('teleop')

    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # -------------------- ADDED (minimal) : subscribe /scan --------------------
    latest_scan = None

    def scan_cb(msg):
        nonlocal latest_scan
        latest_scan = msg

    node.create_subscription(LaserScan, '/scan', scan_cb, qos_profile_sensor_data)
    # --------------------------------------------------------------------------

    print("""
╔════════════════════════════════════════╗
║  ROBOT TELEOP - Keyboard Control       ║
╠════════════════════════════════════════╣
║  W / ↑  : Forward                      ║
║  S / ↓  : Backward                     ║
║  A / ←  : Rotate Left                  ║
║  D / →  : Rotate Right                 ║
║  SPACE  : Stop                         ║
║  Q      : Quit                         ║
║----------------------------------------║
║  M      : Toggle AUTO / MANUAL         ║
║  R      : AUTO follow RIGHT wall       ║
║  L      : AUTO follow LEFT wall        ║
╚════════════════════════════════════════╝
    """)

    twist = Twist()

    # -------------------- ADDED (minimal) : AUTO state + params --------------------
    auto_enabled = False
    follow_side = "right"   # "right" or "left"

    # If your scan "front" isn't at 0°, set this offset after checking RViz (deg)
    idx_front_deg = 0.0

    theta_deg = 50.0
    lookahead_L = 0.35
    d_ref = 0.45

    kp = 0.8   # was 2.0 → reduced for stability
    kd = 0.6   # was 0.4 → increased for better damping

    # ===== FIXED SPEED LIMITS (was dangerous) =====
    v_max = 0.50     # was 0.60 → reduced for safety
    v_min = 0.10     # was 0.15 → reduced for fine control
    w_max = 0.50     # was 2.5 (143°/s!) → reduced to 28°/s for SLAM stability
    w_emergency = 1.0 # emergency-only: 57°/s for escape
    k_turn = 1.0

    # ===== FIXED SAFETY DISTANCES (was collision-prone) =====
    d_stop = 0.35   # was 0.25 → +40% buffer for reaction time
    d_slow = 0.80   # was 0.60 → +33% for smoother braking

    # ===== FIXED SAFETY TIMING =====
    emergency_hold_time = 0.8  # was 0.35 → +130% for proper escape

    # ===== FIXED LIDAR COVERAGE =====
    front_half_deg = 15.0  # was 10.0 → +50% to catch side obstacles
    fallback_half_deg = 25.0  # was 20.0 → +25% for safety

    e_prev = 0.0
    t_prev = time.time()

    def clamp(x, lo, hi):
        return max(lo, min(hi, x))
    # ---------------------------------------------------------------------------

    # -------------------- ADDED (minimal) : EMERGENCY LATCH (stop + turn) -------
    emergency_active = False
    emergency_until = 0.0
    # emergency_hold_time = 0.8  # MOVED TO TOP PARAMS
    # ---------------------------------------------------------------------------

    try:
        while True:
            # -------------------- ADDED (minimal): allow ROS callbacks --------------------
            rclpy.spin_once(node, timeout_sec=0.0)
            # ---------------------------------------------------------------------------

            # -------------------- ADDED (minimal): AUTO runs periodically --------------------
            if auto_enabled:
                now = time.time()
                if (now - t_prev) >= 0.10:  # ~10Hz
                    dt = now - t_prev
                    t_prev = now

                    # If no scan, stop (do not move blindly)
                    if latest_scan is None or len(latest_scan.ranges) == 0:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        cmd_vel_pub.publish(twist)
                    else:
                        scan = latest_scan
                        N = len(scan.ranges)

                        def sanitize(r):
                            if not math.isfinite(r):
                                return math.inf
                            if r <= scan.range_min or r >= scan.range_max:
                                return math.inf
                            return r

                        def wrap(i):
                            return i % N if N > 0 else 0

                        def angle_to_index(angle_rad):
                            i = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
                            return wrap(i)

                        def range_rel_deg(rel_deg):
                            front_angle = scan.angle_min + math.radians(idx_front_deg)
                            angle = front_angle + math.radians(rel_deg)
                            idx = angle_to_index(angle)
                            return sanitize(scan.ranges[idx])

                        def sector_min_rel_deg(rel_center_deg, half_deg):
                            half_steps = max(1, int(round(math.radians(half_deg) / scan.angle_increment)))
                            front_angle = scan.angle_min + math.radians(idx_front_deg)
                            center_angle = front_angle + math.radians(rel_center_deg)
                            center_idx = angle_to_index(center_angle)
                            best = math.inf
                            for di in range(-half_steps, half_steps + 1):
                                r = sanitize(scan.ranges[wrap(center_idx + di)])
                                if r < best:
                                    best = r
                            return best

                        def emergency_turn_sign():
                            # Compare front-left and front-right free space
                            left_free = sector_min_rel_deg(+45.0, 10.0)
                            right_free = sector_min_rel_deg(-45.0, 10.0)
                            return +1.0 if left_free > right_free else -1.0

                        # 1) Front safety measurement
                        d_front = sector_min_rel_deg(0.0, front_half_deg)

                        # --------------- EMERGENCY LATCH (STOP + TURN) ----------------
                        # Trigger emergency if obstacle dangerously close
                        if d_front < d_stop:
                            emergency_active = True
                            emergency_until = time.time() + emergency_hold_time

                        if emergency_active:
                            # Keep emergency behavior at least emergency_hold_time,
                            # then only release when front is clearly open again.
                            if (time.time() < emergency_until) or (d_front < d_slow):
                                v = 0.0
                                w = emergency_turn_sign() * w_emergency
                                twist.linear.x = float(v)
                                twist.angular.z = float(w)
                                cmd_vel_pub.publish(twist)
                                # IMPORTANT: skip normal control this cycle
                                # so nothing can override emergency
                                # (this is the key fix)
                                # --------------------------------------------------
                                # continue to next loop iteration
                                # --------------------------------------------------
                                # NOTE: keep dt/e_prev unchanged here (fine)
                                continue
                            else:
                                emergency_active = False
                        # -----------------------------------------------------------

                        # 2) Two-beam wall follow (normal mode)
                        theta = math.radians(theta_deg)

                        if follow_side == "right":
                            rel_b = -90.0
                            rel_a = -90.0 + theta_deg
                            sign = +1.0  # too close => turn left
                        else:
                            rel_b = +90.0
                            rel_a = +90.0 - theta_deg
                            sign = -1.0  # mirror

                        b = range_rel_deg(rel_b)
                        a = range_rel_deg(rel_a)
                        valid = math.isfinite(a) and math.isfinite(b)

                        if valid:
                            num = a * math.cos(theta) - b
                            den = a * math.sin(theta)
                            alpha = math.atan2(num, den)

                            d = b * math.cos(alpha)
                            d_future = d + lookahead_L * math.sin(alpha)

                            e = d_ref - d_future
                            de = (e - e_prev) / dt if dt > 1e-3 else 0.0
                            e_prev = e

                            w = sign * (kp * e + kd * de)
                            w = clamp(w, -w_max, +w_max)
                        else:
                            # fallback: sector wall distance
                            d_side = sector_min_rel_deg(rel_b, fallback_half_deg)
                            if not math.isfinite(d_side):
                                w = 0.0
                            else:
                                e = d_ref - d_side
                                de = (e - e_prev) / dt if dt > 1e-3 else 0.0
                                e_prev = e
                                w = sign * (kp * e + kd * de)
                                w = clamp(w, -w_max, +w_max)

                        # 3) Speed scheduling (front + turn)
                        v_front = v_max * clamp((d_front - d_stop) / (d_slow - d_stop), 0.0, 1.0)
                        v_turn = v_max / (1.0 + k_turn * abs(w))
                        v = min(v_front, v_turn)
                        v = clamp(v, v_min, v_max)

                        twist.linear.x = float(v)
                        twist.angular.z = float(w)
                        cmd_vel_pub.publish(twist)
            # ---------------------------------------------------------------------------

            # -------------------- ADDED (minimal): non-blocking key check --------------------
            # Only call get_key() when a key is available, otherwise keep looping.
            # Use 0.0 to reduce latency; add tiny sleep to avoid 100% CPU.
            rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not rlist:
                time.sleep(0.005)
                continue
            key = get_key()
            # ---------------------------------------------------------------------------

            # -------------------- ADDED: If emergency active, ignore motion keys ------------
            # Allow only STOP and QUIT keys to pass through.
            if emergency_active and key not in [' ', 'q', 'Q']:
                print("⚠️ SAFETY (emergency) active: command ignored")
                continue
            # ---------------------------------------------------------------------------

            # -------------------- ORIGINAL manual teleop behavior (unchanged) ----------------
            if key.upper() == 'W':
                if not auto_enabled:
                    twist.linear.x = 0.3
                    twist.angular.z = 0.0
                    print("→ FORWARD")
                    cmd_vel_pub.publish(twist)

            elif key.upper() == 'S':
                if not auto_enabled:
                    twist.linear.x = -0.3
                    twist.angular.z = 0.0
                    print("← BACKWARD")
                    cmd_vel_pub.publish(twist)

            elif key.upper() == 'A':
                if not auto_enabled:
                    twist.linear.x = 0.0
                    twist.angular.z = 1.5  # Increased from 0.5
                    print("↻ ROTATE LEFT")
                    cmd_vel_pub.publish(twist)

            elif key.upper() == 'D':
                if not auto_enabled:
                    twist.linear.x = 0.0
                    twist.angular.z = -1.5  # Increased from 0.5
                    print("↺ ROTATE RIGHT")
                    cmd_vel_pub.publish(twist)

            elif key == ' ':
                # STOP always works + disables AUTO + clears emergency
                auto_enabled = False
                emergency_active = False
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                cmd_vel_pub.publish(twist)
                print("⊙ STOP")

            elif key.upper() == 'Q':
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                cmd_vel_pub.publish(twist)
                print("Quitting...")
                break

            # -------------------- ADDED (minimal): AUTO controls (same if/elif style) --------
            elif key.upper() == 'M':
                auto_enabled = not auto_enabled

                # Always STOP immediately when toggling mode to avoid using stale twist
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                cmd_vel_pub.publish(twist)

                # Reset PD state to avoid derivative spike
                e_prev = 0.0
                t_prev = time.time()

                if not auto_enabled:
                    print("MANUAL mode")
                else:
                    print("AUTO mode (Two-Beam)")

            elif key.upper() == 'R':
                follow_side = "right"
                print("AUTO wall: RIGHT")

            elif key.upper() == 'L':
                follow_side = "left"
                print("AUTO wall: LEFT")
            # ---------------------------------------------------------------------------

            else:
                continue

    except KeyboardInterrupt:
        print("\nStopping...")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
