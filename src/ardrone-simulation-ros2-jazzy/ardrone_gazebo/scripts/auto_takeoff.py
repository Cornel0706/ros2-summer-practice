#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

class AutoTakeoff(Node):
    def __init__(self):
        super().__init__('auto_takeoff')
        
        # Subscribe to odom to verify the simulation is running (relative topic)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Publish takeoff command (relative topic for namespace support)
        self.pub = self.create_publisher(Empty, 'ardrone/takeoff', 10)
        self.timer = None
        self.odom_received = False
        self.pub_count = 0
        
        self.get_logger().info('Auto Takeoff node initialized. Waiting for odometry...')

    def odom_callback(self, msg):
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info('Odometry received! Starting 5-second takeoff countdown...')
            # Start timer (sim time) to publish takeoff after 5 seconds
            self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.pub_count += 1
        if self.pub_count >= 5 and self.pub_count <= 10:
            self.get_logger().info(f'Publishing takeoff command ({self.pub_count - 4}/6)...')
            self.pub.publish(Empty())
        elif self.pub_count > 10:
            self.get_logger().info('Takeoff sequence complete. Exiting.')
            if self.timer:
                self.timer.cancel()
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = AutoTakeoff()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
