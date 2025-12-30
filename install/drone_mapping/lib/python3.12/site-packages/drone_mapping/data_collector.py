import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',  # Adjust topic as needed
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Data Collector Node has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received point cloud data: {len(msg.data)} bytes')
        # TODO: Implement saving logic (e.g. to pcd file or accumulating)

def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
