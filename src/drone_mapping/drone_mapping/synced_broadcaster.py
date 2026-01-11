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
import cv2
from cv_bridge import CvBridge

class SyncedBroadcaster(Node):
    def __init__(self):
        super().__init__('synced_broadcaster')
        
        # Tools
        self.bridge = CvBridge()
        
        # 1. Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish Static TF once at startup (Latched)
        self.publish_static_tf()
        
        # 2. Publishers
        self.depth_pub = self.create_publisher(Image, '/camera/depth_synced', 10)
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb_synced', 10)
        self.odom_pub = self.create_publisher(Odometry, '/camera/odom_synced', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info_synced', 10)

        # 3. State: Odom Buffer (Time -> Msg)
        # Store tuples of (nanoseconds, msg)
        # 3. State: Odom Buffer (Time -> Msg)
        # Store tuples of (nanoseconds, msg)
        self.odom_buffer = collections.deque(maxlen=100)
        self.rgb_buffer = collections.deque(maxlen=100)
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

        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/rgb',
            self.rgb_callback,
            qos_profile_sensor_data
        )

        self.gt_buffer = collections.deque(maxlen=100)
        self.use_ground_truth = True 

        # 6. Subscribers
        self.gt_sub = self.create_subscription(
            Odometry,
            '/ground_truth/odom',
            self.gt_callback,
            qos_profile_sensor_data
        )
        self.get_logger().info('Synced Broadcaster: Odom Interpolation Enabled (Ground Truth Priority)')

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
        
        # Send all
        self.static_broadcaster.sendTransform([t_mount, t_optical])




    def gt_callback(self, msg):
        t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.gt_buffer.append((t_ns, msg))

    def odom_callback(self, msg):
        # Store (time_in_ns, msg)
        t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.odom_buffer.append((t_ns, msg))

    def rgb_callback(self, msg):
        # Simply forward the RGB image but sync its timestamp
        # We NO LONGER resize to 640x480. We trust the source resolution.
        
        # Store (time_in_ns, msg)
        t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.rgb_buffer.append((t_ns, msg))




    def normalize_quaternion(self, q):
        norm = (q.x**2 + q.y**2 + q.z**2 + q.w**2)**0.5
        if norm == 0:
            return q
        q.x /= norm
        q.y /= norm
        q.z /= norm
        q.w /= norm
        return q

    def get_interpolated_odom(self, target_time_ns):
        # Decide which buffer to use
        if self.use_ground_truth:
            buffer = self.gt_buffer
        else:
            buffer = self.odom_buffer

        if not buffer:
            return None

        # If only one message, use it
        if len(self.odom_buffer) == 1:
            return self.odom_buffer[0][1]

        # Extract timestamps for binary search
        times = [t for t, m in buffer]
        
        # Find insertion point
        idx = bisect.bisect_left(times, target_time_ns)

        # Exact match or beyond last element (use last)
        if idx == len(times):
            return buffer[-1][1]
        
        # Before first element (use first)
        if idx == 0:
            return buffer[0][1]

        # Interpolate between idx-1 and idx
        t1, msg1 = buffer[idx-1]
        t2, msg2 = buffer[idx]

        # Calculate alpha
        if t2 == t1:
            return msg1
            
        alpha = (target_time_ns - t1) / (t2 - t1)

        # Interpolate Position
        p1 = msg1.pose.pose.position
        p2 = msg2.pose.pose.position
        p2.z = p2.z
        
        # Create interpolated Odom-like structure (just what we need)
        # We can re-use msg1 as container but update position/orientation
        from copy import deepcopy
        start_msg = deepcopy(msg1) # Safer to copy
        
        start_msg.pose.pose.position.x = p1.x + alpha * (p2.x - p1.x)
        start_msg.pose.pose.position.y = p1.y + alpha * (p2.y - p1.y)
        start_msg.pose.pose.position.z = p1.z + alpha * (p2.z - p1.z)
        
        # N-LERP for Orientation (Normalize after Lerp)
        q1 = msg1.pose.pose.orientation
        q2 = msg2.pose.pose.orientation
        
        start_msg.pose.pose.orientation.x = q1.x + alpha * (q2.x - q1.x)
        start_msg.pose.pose.orientation.y = q1.y + alpha * (q2.y - q1.y)
        start_msg.pose.pose.orientation.z = q1.z + alpha * (q2.z - q1.z)
        start_msg.pose.pose.orientation.w = q1.w + alpha * (q2.w - q1.w)
        
        # CRITICAL: Normalize to ensure valid rotation (fixes "dips")
        start_msg.pose.pose.orientation = self.normalize_quaternion(start_msg.pose.pose.orientation)
        
        return start_msg

    def get_closest_rgb(self, target_time_ns):
        # Simple closest search
        if not self.rgb_buffer:
            return None
            
        best_msg = None
        min_diff = float('inf')
        
        # Search recent buffer (optimization: check from end)
        # For now, linear scan is fine for small buffer (100)
        # Or binary search if we trust monotonicity (we should)
        
        times = [t for t, m in self.rgb_buffer]
        idx = bisect.bisect_left(times, target_time_ns)
        
        candidates = []
        if idx < len(times):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)
            
        for i in candidates:
            diff = abs(times[i] - target_time_ns)
            if diff < min_diff:
                min_diff = diff
                best_msg = self.rgb_buffer[i][1]
                
        # threshold: e.g. 33ms (1 frame at 30fps)
        if min_diff > 0.05 * 1e9: # 50ms tolerance
            return None
            
        return best_msg

    def depth_callback(self, msg):
        target_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        
        best_odom = self.get_interpolated_odom(target_ns)
        best_rgb = self.get_closest_rgb(target_ns)
        
        if best_odom is None or best_rgb is None:
            return
            
        # We need Camera Info to be valid
        if self.latest_camera_info is None:
            return
            
        t = TransformStamped()
        
        # CRITICAL: Use the Image's timestamp
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = best_odom.pose.pose.position.x
        t.transform.translation.y = best_odom.pose.pose.position.y
        t.transform.translation.z = best_odom.pose.pose.position.z
        t.transform.rotation = best_odom.pose.pose.orientation
        
        t.transform.translation.z = best_odom.pose.pose.position.z
        t.transform.rotation = best_odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
        
        # Forward image (Original Header)
        msg.header.frame_id = 'camera_link_optical'
        self.depth_pub.publish(msg)
        
        # Publish Synced Odom (Exact same timestamp)
        # We must update header to match image
        best_odom.header.stamp = msg.header.stamp
        # Frame remains 'odom' -> 'base_link'
        best_odom.header.frame_id = 'odom'
        self.odom_pub.publish(best_odom)
        
        # Forward Info (Synced Header)
        info_msg = self.latest_camera_info
        info_msg.header.stamp = msg.header.stamp
        info_msg.header.frame_id = 'camera_link_optical'
        self.info_pub.publish(info_msg)

        # Forward Synced RGB (Same header as Depths)
        best_rgb.header.stamp = msg.header.stamp
        best_rgb.header.frame_id = 'camera_link_optical'
        self.rgb_pub.publish(best_rgb)



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
