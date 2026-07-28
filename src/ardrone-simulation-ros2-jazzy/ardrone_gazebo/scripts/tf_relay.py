#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage

class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        
        # Subscribe to global /tf
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            100)
            
        # Publish to local tf
        self.publisher = self.create_publisher(TFMessage, 'tf', 100)
        
        # QoS for static tf (Transient Local)
        qos_profile = QoSProfile(depth=100)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        
        # Subscribe to global /tf_static
        self.subscription_static = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_static_callback,
            qos_profile)
            
        # Publish to local tf_static
        self.publisher_static = self.create_publisher(TFMessage, 'tf_static', qos_profile)

    def tf_callback(self, msg):
        self.publisher.publish(msg)

    def tf_static_callback(self, msg):
        self.publisher_static.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
