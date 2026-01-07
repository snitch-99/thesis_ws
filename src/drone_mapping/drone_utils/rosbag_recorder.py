#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from drone_interfaces.msg import SyncedPointCloud

import collections
import bisect
from copy import deepcopy

class RosbagRecorder(Node):
    def __init__(self):
        super().__init__('rosbag_recorder')
        
        # 1. Buffers
        # Odom Buffer: Dict {time_ns: msg}
        # Since timestamps are identical, we just match keys.
        self.odom_buffer = {} 
        self.max_buffer_size = 100
        
        # 2. Subscribers
        # Listen to the SYNCED odom (already interpolated by broadcaster)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/camera/odom_synced',
            self.odom_callback,
            10
        )
        
        self.points_sub = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.points_callback,
            10
        )
        
        # 3. Publisher
        self.synced_pub = self.create_publisher(
            SyncedPointCloud, 
            '/camera/synced_packet', 
            10
        )
        
        self.get_logger().info('Rosbag Recorder: Initialized (Exact Sync Mode). Waiting for data...')

    def odom_callback(self, msg):
        # Store by exact timestamp
        t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.odom_buffer[t_ns] = msg
        
        # Simple cleanup if buffer grows too large
        if len(self.odom_buffer) > self.max_buffer_size:
            # Remove oldest key (approximate cleanup is fine)
            oldest_key = min(self.odom_buffer.keys())
            del self.odom_buffer[oldest_key]

    def points_callback(self, msg):
        # 1. Get Cloud Timestamp
        cloud_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        
        # 2. Exact Lookup
        if cloud_ns in self.odom_buffer:
            matched_odom = self.odom_buffer.pop(cloud_ns) # Pop to clean up
            
            # 3. Create Bundle
            synced_msg = SyncedPointCloud()
            synced_msg.header = msg.header
            synced_msg.odom = matched_odom
            synced_msg.point_cloud = msg
            
            # 4. Publish
            self.synced_pub.publish(synced_msg)
        else:
            # Logic: If Odom hasn't arrived yet (msg order), we might drop it.
            # In practice, Broadcaster publishes Odom then Image, 
            # DepthProc takes time, so Cloud arrives AFTER Odom.
            # So Odom should be in buffer.
            # If not, it means we missed the pairing.
            # self.get_logger().warn(f'No matching Odom for cloud at {cloud_ns}')
            pass

def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
