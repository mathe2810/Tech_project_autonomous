#!/usr/bin/env python3
"""Wait for a ROS2 topic to be available"""

import sys
import time
import rclpy
from rclpy.node import Node

class TopicWaiter(Node):
    def __init__(self, topic_name, timeout=10):
        super().__init__('topic_waiter', allow_undeclared_parameters=True)
        self.topic_name = topic_name
        self.timeout = timeout
        self.found = False
        
    def wait(self):
        start_time = time.time()
        while (time.time() - start_time) < self.timeout:
            try:
                topic_names_and_types = self.get_topic_names_and_types()
                topics = [name for name, _ in topic_names_and_types]
                if self.topic_name in topics:
                    print(f"✓ Topic '{self.topic_name}' found", flush=True)
                    return True
            except Exception:
                pass
            
            time.sleep(0.2)
        
        print(f"❌ Topic '{self.topic_name}' not found after {self.timeout}s", flush=True)
        return False

if __name__ == '__main__':
    topic = sys.argv[1] if len(sys.argv) > 1 else '/scan'
    timeout = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    
    rclpy.init()
    waiter = TopicWaiter(topic, timeout)
    
    success = waiter.wait()
    waiter.destroy_node()
    rclpy.shutdown()
    
    sys.exit(0 if success else 1)
