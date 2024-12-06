import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from controls_movement.thruster_allocator import ThrustAllocator

class IntListPublisher(Node):
    def __init__(self):
        super().__init__('int_list_publisher')
        
        # Create a publisher that publishes to the 'thruster_control' topic
        self.publisher_ = self.create_publisher(Int32MultiArray, 'thruster_control', 10)
        
        # Create a timer that triggers the publish callback at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_message)
        
        # Initialize the ThrustAllocator and generate the thrust list
        self.thrust_allocator = ThrustAllocator()
        self.int_list = self.thrust_allocator.getThrustPwm([10, 0, 0], [10, 10, 10])
        
        self.get_logger().info("IntListPublisher initialized and ready to publish.")

    def publish_message(self):
        self.int_list = [int(max(min(val, 2147483647), -2147483648)) for val in self.int_list]
        # Create and publish the message
        msg = Int32MultiArray()
        msg.data = self.int_list
        
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create an instance of the IntListPublisher node
    node = IntListPublisher()
    
    try:
        # Spin the node to keep it active
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down the IntListPublisher node.")
    finally:
        # Destroy the node explicitly and shutdown rclpy
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

"""
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')   #this is the name of the node
        self.publisher_ = self.create_publisher(ChatMessage, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = ChatMessage()
        msg.name = "user1"
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    """