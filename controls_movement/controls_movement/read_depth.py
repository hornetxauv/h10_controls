import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DepthReader(Node):
    def __init__(self):
        super().__init__('depth_reader')
        self.depth = None  # Store the latest depth value here
        self.subscription = self.create_subscription(
            Float32,
            '/sensors/depth',
            self.depth_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def depth_callback(self, msg):
        self.depth = msg.data  # Update the depth value
        self.get_logger().info(f"Depth: {self.depth}")

    def get_depth(self):
        return self.depth

def main(args=None):
    rclpy.init(args=args)
    depth_reader = DepthReader()
    rclpy.spin(depth_reader)
    depth_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()