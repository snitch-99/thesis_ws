#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo

import collections
import bisect

class SyncedBroadcaster(Node):
    def __init__(self):
        super().__init__('synced_broadcaster')
        
        # 1. Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish Static TF once at startup (Latched)
        self.publish_static_tf()
        
        # 2. Publishers
        # 2. Publishers
        self.depth_pub = self.create_publisher(Image, '/camera/depth_synced', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info_synced', 10)

        # 3. State: Odom Buffer (Time -> Msg)
        # Store tuples of (nanoseconds, msg)
        # 3. State: Odom Buffer (Time -> Msg)
        # Store tuples of (nanoseconds, msg)
        self.odom_buffer = collections.deque(maxlen=100)
        self.latest_camera_info = None

        # 4. JSON QoS for Odom (Best Effort)
        from rclpy.qos import qos_profile_sensor_data
        
        # 5. Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth',
            self.depth_callback,
            10
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('Synced Broadcaster: Odom Interpolation Enabled')

    def publish_static_tf(self):
        # 1. Physical Mount (base_link -> camera_link)
        t_mount = TransformStamped()
        t_mount.header.stamp.sec = 0
        t_mount.header.stamp.nanosec = 0
        t_mount.header.frame_id = 'base_link'
        t_mount.child_frame_id = 'camera_link'
        
        # Physical Installation Position
        t_mount.transform.translation.x = 0.13233 
        t_mount.transform.translation.y = 0.0
        t_mount.transform.translation.z = 0.26078
        
        # Keep camera_link aligned with base_link (X-Forward, Z-Up)
        t_mount.transform.rotation.x = 0.0
        t_mount.transform.rotation.y = 0.0
        t_mount.transform.rotation.z = 0.0
        t_mount.transform.rotation.w = 1.0

        # 2. Optical Frame (camera_link -> camera_link_optical)
        t_optical = TransformStamped()
        t_optical.header.stamp.sec = 0
        t_optical.header.stamp.nanosec = 0
        t_optical.header.frame_id = 'camera_link'
        t_optical.child_frame_id = 'camera_link_optical'
        
        t_optical.transform.translation.x = 0.0
        t_optical.transform.translation.y = 0.0
        t_optical.transform.translation.z = 0.0
        
        # Standard Optical Rotation (-0.5, 0.5, -0.5, 0.5)
        # Rotates Look-At-Z (Optical) to Look-At-X (Body)
        t_optical.transform.rotation.x = -0.5
        t_optical.transform.rotation.y = 0.5
        t_optical.transform.rotation.z = -0.5
        t_optical.transform.rotation.w = 0.5
        
        # Send both
        self.static_broadcaster.sendTransform([t_mount, t_optical])

    def odom_callback(self, msg):
        # Store (time_in_ns, msg)
        t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.odom_buffer.append((t_ns, msg))

    def get_interpolated_odom(self, target_time_ns):
        if not self.odom_buffer:
            return None

        # If only one message, use it
        if len(self.odom_buffer) == 1:
            return self.odom_buffer[0][1]

        # Extract timestamps for binary search
        times = [t for t, m in self.odom_buffer]
        
        # Find insertion point
        idx = bisect.bisect_left(times, target_time_ns)

        # Exact match or beyond last element (use last)
        if idx == len(times):
            return self.odom_buffer[-1][1]
        
        # Before first element (use first)
        if idx == 0:
            return self.odom_buffer[0][1]

        # Interpolate between idx-1 and idx
        t1, msg1 = self.odom_buffer[idx-1]
        t2, msg2 = self.odom_buffer[idx]

        # Calculate alpha
        if t2 == t1:
            return msg1
            
        alpha = (target_time_ns - t1) / (t2 - t1)

        # Interpolate Position
        p1 = msg1.pose.pose.position
        p2 = msg2.pose.pose.position
        
        # Create interpolated Odom-like structure (just what we need)
        # We can re-use msg1 as container but update position/orientation
        from copy import deepcopy
        start_msg = deepcopy(msg1) # Safer to copy
        
        start_msg.pose.pose.position.x = p1.x + alpha * (p2.x - p1.x)
        start_msg.pose.pose.position.y = p1.y + alpha * (p2.y - p1.y)
        start_msg.pose.pose.position.z = p1.z + alpha * (p2.z - p1.z)
        
        # Slerp for Orientation would be better, but Lerp is acceptable for small deltas
        # (Assuming 50Hz Odom, deltas are small)
        q1 = msg1.pose.pose.orientation
        q2 = msg2.pose.pose.orientation
        
        start_msg.pose.pose.orientation.x = q1.x + alpha * (q2.x - q1.x)
        start_msg.pose.pose.orientation.y = q1.y + alpha * (q2.y - q1.y)
        start_msg.pose.pose.orientation.z = q1.z + alpha * (q2.z - q1.z)
        start_msg.pose.pose.orientation.w = q1.w + alpha * (q2.w - q1.w)
        
        return start_msg

    def depth_callback(self, msg):
        target_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        
        best_odom = self.get_interpolated_odom(target_ns)
        
        if best_odom is None:
            return
            
        # We need Camera Info to be valid
        if self.latest_camera_info is None:
            return
            
        t = TransformStamped()
        
        # CRITICAL: Use the Image's timestamp
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = best_odom.pose.pose.position.x
        t.transform.translation.y = best_odom.pose.pose.position.y
        t.transform.translation.z = best_odom.pose.pose.position.z
        t.transform.rotation = best_odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
        # Forward image (Original Header)
        msg.header.frame_id = 'camera_link_optical'
        self.depth_pub.publish(msg)
        
        # Forward Info (Synced Header)
        info_msg = self.latest_camera_info
        info_msg.header.stamp = msg.header.stamp
        info_msg.header.frame_id = 'camera_link_optical'
        self.info_pub.publish(info_msg)

    def info_callback(self, msg):
        self.latest_camera_info = msg

def main(args=None):
    rclpy.init(args=args)
    node = SyncedBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
