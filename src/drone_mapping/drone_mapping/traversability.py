import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TraversabilityNode(Node):
    def __init__(self):
        super().__init__('traversability_node')
        
        # Publisher for local position setpoints
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10)
        
        # Timer for 10Hz control loop
        self.timer = self.create_timer(0.1, self.timer_cb)
        
        self.get_logger().info('Traversability Node Started: Sending setpoints at 10Hz')

    def timer_cb(self):
        # Create a simple hover setpoint for testing
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.0  # Hover at 2 meters
        
        self.local_pos_pub.publish(pose)

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
