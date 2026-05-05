#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

import collections
import bisect


class SyncedBroadcaster(Node):
    def __init__(self):
        super().__init__('synced_broadcaster')

        # Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Publish Static TF once at startup (Latched)
        self.publish_static_tf()

        # Publishers — just frame_id corrected passthrough
        self.depth_pub = self.create_publisher(Image, '/camera/depth_corrected', 10)
        self.rgb_pub = self.create_publisher(Image, '/camera/rgb_corrected', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info_corrected', 10)

        # Odom buffers for TF interpolation
        self.odom_buffer = collections.deque(maxlen=100)
        self.gt_buffer = collections.deque(maxlen=100)
        self.use_ground_truth = True

        from rclpy.qos import qos_profile_sensor_data

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/mavros/local_position/odom',
            self.odom_callback, qos_profile_sensor_data)

        self.gt_sub = self.create_subscription(
            Odometry, '/ground_truth/odom',
            self.gt_callback, qos_profile_sensor_data)

        self.depth_sub = self.create_subscription(
            Image, '/camera/depth',
            self.depth_callback, 10)

        self.rgb_sub = self.create_subscription(
            Image, '/camera/rgb',
            self.rgb_callback, qos_profile_sensor_data)

        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info',
            self.info_callback, qos_profile_sensor_data)

        self.get_logger().info('Synced Broadcaster: TF + frame_id correction (Ground Truth Priority)')

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
        t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.odom_buffer.append((t_ns, msg))

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
        if len(buffer) == 1:
            return buffer[0][1]

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

        from copy import deepcopy
        start_msg = deepcopy(msg1)

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

    # ── Helper: publish TF at a given image timestamp ──

    def publish_tf_for_stamp(self, stamp):
        target_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        best_odom = self.get_interpolated_odom(target_ns)
        if best_odom is None:
            return

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = best_odom.pose.pose.position.x
        t.transform.translation.y = best_odom.pose.pose.position.y
        t.transform.translation.z = best_odom.pose.pose.position.z
        t.transform.rotation = best_odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    # ── Image callbacks: publish TF + correct frame_id and republish ──

    def depth_callback(self, msg):
        self.publish_tf_for_stamp(msg.header.stamp)
        msg.header.frame_id = 'camera_link_optical'
        self.depth_pub.publish(msg)

    def rgb_callback(self, msg):
        self.publish_tf_for_stamp(msg.header.stamp)
        msg.header.frame_id = 'camera_link_optical'
        self.rgb_pub.publish(msg)

    def info_callback(self, msg):
        """Forward camera_info with corrected frame_id."""
        msg.header.frame_id = 'camera_link_optical'
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyncedBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
