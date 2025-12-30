import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class MavrosControlNode(Node):
    def __init__(self):
        super().__init__('mavros_control_node')
        
        self.current_state = State()
        self.last_req = self.get_clock().now()
        
        # Subscriber for state
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            10)
            
        # Service clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        
        # Wait for services
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arming service not available, waiting again...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')

        # Timer for checking state (e.g., 5Hz)
        self.timer = self.create_timer(0.2, self.timer_cb)
        self.get_logger().info('Mavros Control Node Started')

    def state_cb(self, msg):
        self.current_state = msg

    def timer_cb(self):
        now = self.get_clock().now()
        # Only try to arm/offboard every 5 seconds to avoid flooding
        if (now - self.last_req).nanoseconds > 5 * 1e9:
            if self.current_state.mode != "OFFBOARD":
                self.set_mode("OFFBOARD")
            elif not self.current_state.armed:
                self.arm_drone(True)
            self.last_req = now

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        # We don't block here, just log if needed in a callback or assume it works
        self.get_logger().info(f"Attempting to set mode to {mode}")

    def arm_drone(self, arm):
        req = CommandBool.Request()
        req.value = arm
        future = self.arming_client.call_async(req)
        self.get_logger().info(f"Attempting to {'Arm' if arm else 'Disarm'}")

def main(args=None):
    rclpy.init(args=args)
    node = MavrosControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
