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
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import sys
import termios
import tty
import select
import math
import time


def get_key():
    """Get a single key press"""
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        return ch
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))


def main():
    rclpy.init()
    node = rclpy.create_node('teleop')

    cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # -------------------- subscribe /scan --------------------
    latest_scan = None

    def scan_cb(msg):
        nonlocal latest_scan
        latest_scan = msg

    node.create_subscription(LaserScan, '/scan', scan_cb, qos_profile_sensor_data)
    # ---------------------------------------------------------

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

    # -------------------- AUTO state + params --------------------
    auto_enabled = False
    follow_side = "right"

    # If scan "front" isn't at 0°, set this offset after checking RViz (deg)
    idx_front_deg = 0.0

    theta_deg = 50.0
    lookahead_L = 0.35
    d_ref = 0.45

    kp = 1.2    
    kd = 0.15   

    v_max = 0.25
    v_min = 0.08
    w_max = 0.9 
    k_turn = 1.6

    d_stop = 0.30  
    d_slow = 0.75 

    front_half_deg = 12.0
    fallback_half_deg = 25.0

    e_prev = 0.0
    t_prev = time.time()

    def clamp(x, lo, hi):
        return max(lo, min(hi, x))
    # ------------------------------------------------------------

    # -------------------- EMERGENCY LATCH (stable) ---------------
    emergency_active = False
    emergency_until = 0.0
    emergency_hold_time = 0.60  # was 0.35 (hold longer => stable)
    # Add a "cooldown" after emergency to avoid instant acceleration
    cooldown_until = 0.0
    cooldown_time = 0.80
    # ------------------------------------------------------------

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.0)

            # -------------------- AUTO loop (~10Hz) --------------------
            if auto_enabled:
                now = time.time()
                if (now - t_prev) >= 0.10:
                    dt = now - t_prev
                    t_prev = now

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
                            left_free = sector_min_rel_deg(+45.0, 12.0)
                            right_free = sector_min_rel_deg(-45.0, 12.0)
                            return +1.0 if left_free > right_free else -1.0

                        # 1) Front safety
                        d_front = sector_min_rel_deg(0.0, front_half_deg)

                        # --------------- EMERGENCY LATCH (STOP + SLOW TURN) ----------------
                        if d_front < d_stop:
                            emergency_active = True
                            emergency_until = time.time() + emergency_hold_time
                            cooldown_until = time.time() + cooldown_time

                        if emergency_active:
                            # release only if time passed and front is really clear
                            if (time.time() < emergency_until) or (d_front < d_slow):
                                v = 0.0
                                w = emergency_turn_sign() * w_max
                                twist.linear.x = float(v)
                                twist.angular.z = float(w)
                                cmd_vel_pub.publish(twist)
                                continue
                            else:
                                emergency_active = False
                        # -------------------------------------------------------------------

                        # 2) Two-beam wall follow
                        theta = math.radians(theta_deg)

                        if follow_side == "right":
                            rel_b = -90.0
                            rel_a = -90.0 + theta_deg
                            sign = +1.0
                        else:
                            rel_b = +90.0
                            rel_a = +90.0 - theta_deg
                            sign = -1.0

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
                            d_side = sector_min_rel_deg(rel_b, fallback_half_deg)
                            if not math.isfinite(d_side):
                                w = 0.0
                            else:
                                e = d_ref - d_side
                                de = (e - e_prev) / dt if dt > 1e-3 else 0.0
                                e_prev = e
                                w = sign * (kp * e + kd * de)
                                w = clamp(w, -w_max, +w_max)

                        # 3) Speed scheduling (more conservative)
                        v_front = v_max * clamp((d_front - d_stop) / (d_slow - d_stop), 0.0, 1.0)
                        v_turn = v_max / (1.0 + k_turn * abs(w))
                        v = min(v_front, v_turn)

                        # cooldown after emergency: force very low speed
                        if time.time() < cooldown_until:
                            v = min(v, 0.10)

                        v = clamp(v, v_min, v_max)

                        twist.linear.x = float(v)
                        twist.angular.z = float(w)
                        cmd_vel_pub.publish(twist)
            # ---------------------------------------------------------

            # -------------------- non-blocking key check -------------
            rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
            if not rlist:
                time.sleep(0.005)
                continue
            key = get_key()
            # ---------------------------------------------------------

            # If emergency active: ignore motion keys except STOP/Q
            if emergency_active and key not in [' ', 'q', 'Q']:
                print("⚠️ SAFETY active: command ignored")
                continue

            # -------------------- MANUAL teleop behavior (same shape) ----------------
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
                    twist.angular.z = 1.5
                    print("↻ ROTATE LEFT")
                    cmd_vel_pub.publish(twist)

            elif key.upper() == 'D':
                if not auto_enabled:
                    twist.linear.x = 0.0
                    twist.angular.z = -1.5
                    print("↺ ROTATE RIGHT")
                    cmd_vel_pub.publish(twist)

            elif key == ' ':
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

            # -------------------- AUTO controls (same if/elif style) --------
            elif key.upper() == 'M':
                auto_enabled = not auto_enabled

                # Stop immediately when toggling
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                cmd_vel_pub.publish(twist)

                # Reset state
                emergency_active = False
                e_prev = 0.0
                t_prev = time.time()
                cooldown_until = time.time() + 0.3  # small calm start

                if not auto_enabled:
                    print("MANUAL mode")
                else:
                    print("AUTO mode (STABLE mapping)")

            elif key.upper() == 'R':
                follow_side = "right"
                print("AUTO wall: RIGHT")

            elif key.upper() == 'L':
                follow_side = "left"
                print("AUTO wall: LEFT")

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
