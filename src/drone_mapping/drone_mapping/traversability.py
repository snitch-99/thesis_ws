import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from drone_mapping.trajectory_generator import generate_orbit
import math

class TraversabilityNode(Node):
    def __init__(self):
        super().__init__('traversability_node')
        
        # State variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        
        # Flag to track if we have ever successfully entered Offboard/Armed
        self.initial_offboard_triggered = False

        # Orbit Parameters
        self.center_x = 5.0
        self.center_y = -5.0
        self.radius = 7.0
        self.height = 2.0
        self.threshold = 0.2 # meters

        # Generate Waypoints
        # 0.1 radians ~ 5.7 degrees angular resolution
        # Pass Home (0,0) to start orbit at the closest point
        self.waypoints = generate_orbit(
            self.center_x, 
            self.center_y, 
            self.radius, 
            self.height, 
            0.1,
            home_x=0.0,
            home_y=0.0
        )
        self.current_wp_index = 0
        
        # Subscriptions
        # State is usually reliable
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            10)
            
        # Pose is sensor data (Best Effort), so we need a compatible QoS
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.local_pos_sub = self.create_subscription(
            PoseStamped,
            'mavros/local_position/pose',
            self.pose_cb,
            qos_profile)
        
        # Publisher for local position setpoints
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10)
        
        # Timer for 10Hz control loop
        self.timer = self.create_timer(0.1, self.timer_cb)
        
        self.get_logger().info(f'Traversability Node Started: Orbit {len(self.waypoints)} points')

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def timer_cb(self):
        """
        Control Loop:
        - If Armed AND Offboard -> Set Flag, traversing()
        - Else -> no_signal() (Safety Hold)
        """
        if self.current_state.armed and self.current_state.mode == "OFFBOARD":
            self.initial_offboard_triggered = True
            self.traversing()
        else:
            self.no_signal()

    def no_signal(self):
        """
        Safety Hold:
        - If we haven't flown yet (Flag=False), send 0,0,0 (safe default).
        - If we HAVE flown (Flag=True), hold current position (failsafe).
        """
        safety_point = PoseStamped()
        safety_point.header.stamp = self.get_clock().now().to_msg()
        safety_point.header.frame_id = "map"
        
        if self.initial_offboard_triggered:
            # Hold current position
            safety_point.pose.position.x = self.current_pose.pose.position.x
            safety_point.pose.position.y = self.current_pose.pose.position.y
            safety_point.pose.position.z = self.current_pose.pose.position.z
            safety_point.pose.orientation = self.current_pose.pose.orientation
        else:
            # Default to origin
            safety_point.pose.position.x = 0.0
            safety_point.pose.position.y = 0.0
            safety_point.pose.position.z = 2.0
            safety_point.pose.orientation.w = 1.0

        self.local_pos_pub.publish(safety_point)

    def traversing(self):
        """
        Orbit Logic:
        1. Get current target waypoint.
        2. Check distance to target.
        3. If < threshold, advance index.
        4. Calculate Yaw to face (center_x, center_y).
        5. Publish setpoint.
        """
        if not self.waypoints:
            return

        # Get Target Point
        wx, wy, wz = self.waypoints[self.current_wp_index]
        
        # Current Position
        cx = self.current_pose.pose.position.x
        cy = self.current_pose.pose.position.y
        cz = self.current_pose.pose.position.z
        
        # Calculate Distance
        dist = math.sqrt((wx - cx)**2 + (wy - cy)**2 + (wz - cz)**2)
        
        # Advance if close enough
        if dist < self.threshold:
            self.current_wp_index = (self.current_wp_index + 1) % len(self.waypoints)
            # Update target to next point
            wx, wy, wz = self.waypoints[self.current_wp_index]
            self.get_logger().info(f'Reached WP {self.current_wp_index}. Next: {wx:.2f}, {wy:.2f}')

        # Prepare Message
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = "map"
        
        target_pose.pose.position.x = wx
        target_pose.pose.position.y = wy
        target_pose.pose.position.z = wz
        
        # Calculate Yaw to face Center (center_x, center_y)
        # Vector from Drone to Center
        dx = self.center_x - wx  # Ideally calculate from current pos, but from WP is smoother
        dy = self.center_y - wy
        yaw = math.atan2(dy, dx)
        
        # Convert Yaw to Quaternion
        q = self.yaw_to_quaternion(yaw)
        target_pose.pose.orientation = q

        self.local_pos_pub.publish(target_pose)

    def yaw_to_quaternion(self, yaw):
        """
        Convert yaw angle to quaternion (w, x, y, z)
        """
        from geometry_msgs.msg import Quaternion
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
