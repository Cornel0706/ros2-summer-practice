#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import EntityWrench
from nav_msgs.msg import Odometry
import time

class PipelineDiagnostics(Node):
    def __init__(self):
        super().__init__('pipeline_diagnostics')
        
        # Subscriptions to all stages of the cmd_vel pipeline
        self.sub_nav = self.create_subscription(Twist, '/cmd_vel_nav', self.nav_callback, 10)
        self.sub_smoothed = self.create_subscription(Twist, '/cmd_vel_smoothed', self.smoothed_callback, 10)
        self.sub_final = self.create_subscription(Twist, '/cmd_vel', self.final_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.wrench_sub = self.create_subscription(EntityWrench, '/world/empty/wrench', self.wrench_callback, 10)
        
        self.log_file = '/home/cornel/ros2_ws_frontier_detection/src/ardrone-simulation-ros2-jazzy/ardrone_gazebo/diagnostic_log.txt'
        
        self.nav_count = 0
        self.smoothed_count = 0
        self.final_count = 0
        self.odom_count = 0
        self.wrench_count = 0
        
        self.last_nav = None
        self.last_smoothed = None
        self.last_final = None
        
        with open(self.log_file, 'w') as f:
            f.write('Pipeline Diagnostics Started\n')
            
        self.get_logger().info(f'Logging pipeline status to {self.log_file}')
        self.timer = self.create_timer(1.0, self.timer_callback)

    def nav_callback(self, msg):
        self.nav_count += 1
        self.last_nav = msg

    def smoothed_callback(self, msg):
        self.smoothed_count += 1
        self.last_smoothed = msg

    def final_callback(self, msg):
        self.final_count += 1
        self.last_final = msg

    def odom_callback(self, msg):
        self.odom_count += 1

    def wrench_callback(self, msg):
        self.wrench_count += 1

    def timer_callback(self):
        log_lines = []
        log_lines.append('--- Pipeline Status ---')
        log_lines.append(f'Messages received in the last second:')
        log_lines.append(f'  /odom (from Gazebo): {self.odom_count}')
        log_lines.append(f'  /cmd_vel_nav (from ControllerServer): {self.nav_count}')
        log_lines.append(f'  /cmd_vel_smoothed (from VelocitySmoother): {self.smoothed_count}')
        log_lines.append(f'  /cmd_vel (from CollisionMonitor / final output): {self.final_count}')
        log_lines.append(f'  /world/empty/wrench (from Driver): {self.wrench_count}')
        
        if self.last_nav:
            log_lines.append(f'Last cmd_vel_nav values: lin.x={self.last_nav.linear.x:.3f}, ang.z={self.last_nav.angular.z:.3f}')
        if self.last_smoothed:
            log_lines.append(f'Last cmd_vel_smoothed values: lin.x={self.last_smoothed.linear.x:.3f}, ang.z={self.last_smoothed.angular.z:.3f}')
        if self.last_final:
            log_lines.append(f'Last cmd_vel values: lin.x={self.last_final.linear.x:.3f}, ang.z={self.last_final.angular.z:.3f}')
            
        # Reset counters for the next second
        self.odom_count = 0
        self.nav_count = 0
        self.smoothed_count = 0
        self.final_count = 0
        self.wrench_count = 0
        
        log_content = '\n'.join(log_lines) + '\n\n'
        with open(self.log_file, 'a') as f:
            f.write(log_content)

def main(args=None):
    rclpy.init(args=args)
    node = PipelineDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
