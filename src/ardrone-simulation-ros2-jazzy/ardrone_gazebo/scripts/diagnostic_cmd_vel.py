#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import os

class CmdVelDiagnostics(Node):
    def __init__(self):
        super().__init__('cmd_vel_diagnostics')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.log_file = '/home/cornel/ros2_ws_frontier_detection/src/ardrone-simulation-ros2-jazzy/ardrone_gazebo/cmd_vel_log.txt'
        
        # Clear log file on startup
        with open(self.log_file, 'w') as f:
            f.write('cmd_vel Diagnostics Started\n')
            
        self.get_logger().info(f'Logging /cmd_vel to {self.log_file}')

    def callback(self, msg):
        log_str = f'linear: [{msg.linear.x}, {msg.linear.y}, {msg.linear.z}], angular: [{msg.angular.x}, {msg.angular.y}, {msg.angular.z}]\n'
        with open(self.log_file, 'a') as f:
            f.write(log_str)
        self.get_logger().info(f'Received: {log_str.strip()}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
