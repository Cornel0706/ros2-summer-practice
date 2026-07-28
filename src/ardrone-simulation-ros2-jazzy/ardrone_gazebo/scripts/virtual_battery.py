#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import subprocess
import os
import time
import math
from enum import Enum

from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

class State(Enum):
    BOOTING = 0
    EXPLORING = 1
    RETURNING = 2
    RECHARGING = 3
    TAKING_OFF = 4

class VirtualBatteryNode(Node):
    def __init__(self):
        super().__init__('virtual_battery')
        
        self.battery_level = 100
        self.state = State.BOOTING
        
        # Drone position (updated from odometry)
        self.drone_x = 0.0
        self.drone_y = 0.0
        
        # Home coordinates
        self.home_x = 0.0
        self.home_y = 0.0
        
        # Dynamic RTH parameters
        self.drone_max_speed = 0.8       # m/s (from mrtsp_params.yaml)
        self.battery_drain_interval = 3.0 # seconds per 1% drain
        self.safety_margin_percent = 5    # extra % safety buffer
        self.min_rth_threshold = 10       # never go below 10% before RTH
        
        # Action client for Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers for takeoff, landing and reset
        self.land_publisher = self.create_publisher(Empty, '/ardrone/land', 10)
        self.takeoff_publisher = self.create_publisher(Empty, '/ardrone/takeoff', 10)
        self.reset_publisher = self.create_publisher(Empty, '/ardrone/reset', 10)
        
        # Subscribe to odometry for position tracking
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.mrtsp_process = None
        self.takeoff_count = 0
        
        self.get_logger().info("Virtual Battery Node started. Booting (10s)...")
        self.boot_timer = self.create_timer(10.0, self.start_exploration)

    def odom_callback(self, msg):
        self.drone_x = msg.pose.pose.position.x
        self.drone_y = msg.pose.pose.position.y

    def get_distance_to_home(self):
        dx = self.drone_x - self.home_x
        dy = self.drone_y - self.home_y
        return math.sqrt(dx * dx + dy * dy)

    def calculate_dynamic_threshold(self):
        """Calculate the minimum battery % needed to safely return home."""
        distance = self.get_distance_to_home()
        
        # Time needed to fly home (seconds) = distance / speed
        time_to_home = distance / self.drone_max_speed
        
        # Battery % consumed during return = time / drain_interval
        battery_needed = time_to_home / self.battery_drain_interval
        
        # Add safety margin for obstacle avoidance detours
        threshold = battery_needed + self.safety_margin_percent
        
        # Never let threshold drop below minimum
        return max(threshold, self.min_rth_threshold)

    def start_exploration(self):
        if self.boot_timer:
            self.boot_timer.cancel()
            self.boot_timer = None
            
        self.state = State.EXPLORING
        self.get_logger().info("🚀 Drone is active. Smart Battery drain started.")
        self.drain_timer = self.create_timer(self.battery_drain_interval, self.drain_callback)

    def drain_callback(self):
        if self.state != State.EXPLORING:
            return
            
        self.battery_level -= 1
        
        # Calculate dynamic threshold
        distance = self.get_distance_to_home()
        threshold = self.calculate_dynamic_threshold()
        
        # Log battery level every 5%
        if self.battery_level % 5 == 0:
            self.get_logger().info(
                f"⚡ Baterie: {self.battery_level}% | "
                f"Distanța până acasă: {distance:.1f}m | "
                f"Prag RTH dinamic: {threshold:.0f}%"
            )
            
        if self.battery_level <= threshold:
            self.state = State.RETURNING
            self.drain_timer.cancel()
            self.get_logger().warn(
                f"⚠️ SMART RTH TRIGGERED! Baterie: {self.battery_level}% | "
                f"Distanță: {distance:.1f}m | Prag calculat: {threshold:.0f}%"
            )
            self.execute_rth()

    def execute_rth(self):
        # 1. Kill MRTSP Explorer robustly and gracefully
        self.get_logger().info("🛑 Stopping MRTSP explorer process...")
        try:
            if self.mrtsp_process is not None:
                self.mrtsp_process.terminate()
            subprocess.run(['pkill', '-f', 'mrtsp_explorer'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(['pkill', '-f', 'explore.launch.py'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f"Failed to stop mrtsp_explorer: {e}")

        # 2. Send Nav2 goal to home
        self.get_logger().info("⏳ Waiting for navigate_to_pose action server...")
        self.nav_to_pose_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.home_x
        goal_msg.pose.pose.position.y = self.home_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"🏠 Navigating to Home [{self.home_x}, {self.home_y}]...")
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ RTH Goal rejected by Nav2!')
            return

        self.get_logger().info('✅ RTH Goal accepted. Flying home...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        # Status 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info("🏁 Reached Home safely! Initiating Landing...")
        else:
            self.get_logger().warn(f"⚠️ RTH ended with status code: {status}. Landing anyway...")
            
        # 3. Publish land command
        msg = Empty()
        self.land_publisher.publish(msg)
        
        # Transition to Recharging
        self.state = State.RECHARGING
        self.get_logger().info("🔋 Landing successful. Recharging started (15 seconds)...")
        self.recharge_timer = self.create_timer(15.0, self.finish_recharge)

    def finish_recharge(self):
        self.recharge_timer.cancel()
        
        # Reset the drone's internal PID controllers
        self.get_logger().info("🔄 Resetting drone PID controllers...")
        self.reset_publisher.publish(Empty())
        
        self.battery_level = 100
        self.get_logger().info("✅ Baterie reîncărcată la 100%! Pregătire pentru decolare...")
        
        self.state = State.TAKING_OFF
        self.takeoff_count = 0
        self.takeoff_timer = self.create_timer(1.0, self.takeoff_sequence)
        
    def takeoff_sequence(self):
        self.takeoff_count += 1
        if self.takeoff_count <= 5:
            self.get_logger().info(f"🛫 Publishing Takeoff command ({self.takeoff_count}/5)...")
            self.takeoff_publisher.publish(Empty())
        elif self.takeoff_count == 8:
            self.get_logger().info("🌍 Altitude should be stable. Relansare algoritm MRTSP...")
            self.takeoff_timer.cancel()
            self.resume_exploration()

    def resume_exploration(self):
        # Launch mrtsp_explorer again with the drone-specific config
        try:
            from ament_index_python.packages import get_package_share_directory
            params_file = os.path.join(
                get_package_share_directory('ardrone_gazebo'),
                'config', 'mrtsp_params.yaml'
            )
            self.get_logger().info(f"📄 Using MRTSP config: {params_file}")
            self.mrtsp_process = subprocess.Popen([
                'ros2', 'launch', 'mrtsp_exploration_ros2', 'explore.launch.py',
                'use_sim_time:=true',
                f'params_file:={params_file}'
            ])
        except Exception as e:
            self.get_logger().error(f"Eroare la pornirea MRTSP: {e}")
            
        # Go back to Exploring state
        self.state = State.EXPLORING
        self.get_logger().info("🚀 Reluare explorare. Smart Battery drain pornit.")
        self.drain_timer = self.create_timer(self.battery_drain_interval, self.drain_callback)

def main(args=None):
    rclpy.init(args=args)
    node = VirtualBatteryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
