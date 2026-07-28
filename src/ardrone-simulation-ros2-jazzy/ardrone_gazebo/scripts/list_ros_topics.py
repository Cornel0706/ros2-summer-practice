#!/usr/bin/env python3
import rclpy
import time
import os

def main():
    rclpy.init()
    node = rclpy.create_node('graph_inspector')
    
    log_file = '/home/cornel/ros2_ws_frontier_detection/src/ardrone-simulation-ros2-jazzy/ardrone_gazebo/graph_log.txt'
    
    # Wait a bit for graph to settle
    time.sleep(2.0)
    
    topic_names_and_types = node.get_topic_names_and_types()
    
    with open(log_file, 'w') as f:
        f.write("=== ROS 2 Active Topics and Connections ===\n\n")
        for topic_name, topic_types in topic_names_and_types:
            pub_count = node.count_publishers(topic_name)
            sub_count = node.count_subscribers(topic_name)
            f.write(f"Topic: {topic_name}\n")
            f.write(f"  Type: {', '.join(topic_types)}\n")
            f.write(f"  Publishers: {pub_count}\n")
            f.write(f"  Subscribers: {sub_count}\n\n")
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
