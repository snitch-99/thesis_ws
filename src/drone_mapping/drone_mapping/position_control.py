import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class PositionControl(Node):
    def __init__(self):
        super().__init__('position_control')
        self.current_state = State()
        
        # Subscriber for state
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            10)
            
        # Publisher for local position
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10)
            
        # Service clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        
        # Timer for setpoint loop (e.g. 20Hz)
        self.timer = self.create_timer(0.05, self.timer_cb)
        
        self.get_logger().info('Position Control Node has been started.')

    def state_cb(self, msg):
        self.current_state = msg

    def timer_cb(self):
        # Publish a setpoint (e.g. hover at 2m)
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.0
        self.local_pos_pub.publish(pose)

        # TODO: Implement Offboard mode switch and Arming logic here or in a separate triggered method

def main(args=None):
    rclpy.init(args=args)
    node = PositionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
