#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np

class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')

        self.declare_parameter('map1_topic', '/drone1/map')
        self.declare_parameter('map2_topic', '/drone2/map')
        self.declare_parameter('merged_map_topic', '/map')

        map1_topic = self.get_parameter('map1_topic').value
        map2_topic = self.get_parameter('map2_topic').value
        merged_map_topic = self.get_parameter('merged_map_topic').value

        self.map1 = None
        self.map2 = None

        # Maps in ROS 2 use TRANSIENT_LOCAL durability
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub1 = self.create_subscription(OccupancyGrid, map1_topic, self.map1_callback, qos)
        self.sub2 = self.create_subscription(OccupancyGrid, map2_topic, self.map2_callback, qos)

        self.pub = self.create_publisher(OccupancyGrid, merged_map_topic, qos)

    def map1_callback(self, msg):
        self.map1 = msg
        self.merge_and_publish()

    def map2_callback(self, msg):
        self.map2 = msg
        self.merge_and_publish()

    def merge_and_publish(self):
        if self.map1 is None and self.map2 is None:
            return
        
        if self.map1 is None:
            self.pub.publish(self.map2)
            return
            
        if self.map2 is None:
            self.pub.publish(self.map1)
            return

        # We have both maps, need to merge them.
        # Assume same resolution for simplicity, or we can just align them based on origin.
        res = self.map1.info.resolution
        
        # Determine bounding box of both maps
        min_x = min(self.map1.info.origin.position.x, self.map2.info.origin.position.x)
        min_y = min(self.map1.info.origin.position.y, self.map2.info.origin.position.y)
        
        max_x = max(
            self.map1.info.origin.position.x + self.map1.info.width * res,
            self.map2.info.origin.position.x + self.map2.info.width * res
        )
        max_y = max(
            self.map1.info.origin.position.y + self.map1.info.height * res,
            self.map2.info.origin.position.y + self.map2.info.height * res
        )
        
        width = int(np.ceil((max_x - min_x) / res))
        height = int(np.ceil((max_y - min_y) / res))
        
        # Create full grid initialized to -1
        grid1 = np.full((height, width), -1, dtype=np.int8)
        grid2 = np.full((height, width), -1, dtype=np.int8)
        
        # Paste map1 into grid1
        start_x1 = int(round((self.map1.info.origin.position.x - min_x) / res))
        start_y1 = int(round((self.map1.info.origin.position.y - min_y) / res))
        data1 = np.array(self.map1.data, dtype=np.int8).reshape((self.map1.info.height, self.map1.info.width))
        grid1[start_y1:start_y1+self.map1.info.height, start_x1:start_x1+self.map1.info.width] = data1
        
        # Paste map2 into grid2
        start_x2 = int(round((self.map2.info.origin.position.x - min_x) / res))
        start_y2 = int(round((self.map2.info.origin.position.y - min_y) / res))
        data2 = np.array(self.map2.data, dtype=np.int8).reshape((self.map2.info.height, self.map2.info.width))
        grid2[start_y2:start_y2+self.map2.info.height, start_x2:start_x2+self.map2.info.width] = data2
        
        # Merge grids
        # Conditions:
        # If EITHER occupied (>= 65) -> occupied (100)
        # If EITHER free (>= 0 and < 65) and NEITHER occupied -> free (0)
        # Else unknown (-1)
        
        merged_grid = np.full((height, width), -1, dtype=np.int8)
        
        occ_mask1 = grid1 >= 65
        occ_mask2 = grid2 >= 65
        occ_mask = occ_mask1 | occ_mask2
        
        free_mask1 = (grid1 >= 0) & (grid1 < 65)
        free_mask2 = (grid2 >= 0) & (grid2 < 65)
        free_mask = (free_mask1 | free_mask2) & ~occ_mask
        
        merged_grid[free_mask] = 0
        merged_grid[occ_mask] = 100
        
        merged_map = OccupancyGrid()
        merged_map.header.stamp = self.get_clock().now().to_msg()
        # Use latest frame_id from either map, they should be map or global frame
        merged_map.header.frame_id = self.map1.header.frame_id
        
        merged_map.info.resolution = res
        merged_map.info.width = width
        merged_map.info.height = height
        merged_map.info.origin.position.x = min_x
        merged_map.info.origin.position.y = min_y
        merged_map.info.origin.position.z = 0.0
        merged_map.info.origin.orientation.w = 1.0
        
        merged_map.data = merged_grid.flatten().tolist()
        
        self.pub.publish(merged_map)


def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
