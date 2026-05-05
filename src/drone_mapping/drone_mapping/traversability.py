import os
import csv
import json
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from ament_index_python.packages import get_package_share_directory

# ── File paths ──
CONFIG_DIR = os.path.join(get_package_share_directory('drone_mapping'), 'config')
WAYPOINTS_FILE = os.path.join(CONFIG_DIR, 'waypoints.csv')
CONFIG_FILE = os.path.join(CONFIG_DIR, 'trajectory_config.json')


def load_waypoints(csv_path):
    """Load waypoints from CSV file."""
    waypoints = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            waypoints.append((float(row['x']), float(row['y']), float(row['z'])))
    return waypoints


def load_config(json_path):
    """Load trajectory config from JSON file."""
    with open(json_path, 'r') as f:
        return json.load(f)


class TraversabilityNode(Node):
    def __init__(self):
        super().__init__('traversability_node')

        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.initial_offboard_triggered = False

        # Load trajectory config
        config_path = os.path.normpath(CONFIG_FILE)
        self.get_logger().info(f'Loading config from: {config_path}')
        config = load_config(config_path)

        self.center_x = config['center_x']
        self.center_y = config['center_y']
        self.threshold = config.get('threshold', 0.2)

        # Load waypoints
        wp_path = os.path.normpath(WAYPOINTS_FILE)
        self.get_logger().info(f'Loading waypoints from: {wp_path}')
        self.waypoints = load_waypoints(wp_path)
        self.current_wp_index = 0

        # Subscriptions
        self.state_sub = self.create_subscription(
            State, 'mavros/state', self.state_cb, 10)

        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.local_pos_sub = self.create_subscription(
            PoseStamped, 'mavros/local_position/pose', self.pose_cb, qos_profile)

        # Publisher
        self.local_pos_pub = self.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', 10)

        # 10Hz control loop
        self.timer = self.create_timer(0.1, self.timer_cb)

        self.get_logger().info(f'Traversability Node Started: {len(self.waypoints)} waypoints')

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def timer_cb(self):
        if self.current_state.armed and self.current_state.mode == "OFFBOARD":
            self.initial_offboard_triggered = True
            self.traversing()
        else:
            self.no_signal()

    def no_signal(self):
        safety_point = PoseStamped()
        safety_point.header.stamp = self.get_clock().now().to_msg()
        safety_point.header.frame_id = "map"

        if self.initial_offboard_triggered:
            safety_point.pose.position.x = self.current_pose.pose.position.x
            safety_point.pose.position.y = self.current_pose.pose.position.y
            safety_point.pose.position.z = self.current_pose.pose.position.z
            safety_point.pose.orientation = self.current_pose.pose.orientation
        else:
            safety_point.pose.position.x = 0.0
            safety_point.pose.position.y = 0.0
            safety_point.pose.position.z = 2.0
            safety_point.pose.orientation.w = 1.0

        self.local_pos_pub.publish(safety_point)

    def traversing(self):
        if not self.waypoints:
            return

        target_x, target_y, target_z = self.waypoints[self.current_wp_index]

        dx = self.current_pose.pose.position.x - target_x
        dy = self.current_pose.pose.position.y - target_y
        dz = self.current_pose.pose.position.z - target_z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < self.threshold:
            next_index = self.current_wp_index + 1
            # Skip home (index 0) on subsequent orbits
            self.current_wp_index = next_index if next_index < len(self.waypoints) else 1
            target_x, target_y, target_z = self.waypoints[self.current_wp_index]

        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = "map"

        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.position.z = target_z

        yaw = math.atan2(self.center_y - target_y, self.center_x - target_x)
        target_pose.pose.orientation = self.yaw_to_quaternion(yaw)

        self.local_pos_pub.publish(target_pose)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = TraversabilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
