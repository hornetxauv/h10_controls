import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from controls_movement.thruster_allocator import ThrustAllocator

class ThrustPublisher(Node):

    def __init__(self):
        super().__init__('thrust_publisher')

        # Create a publisher for the 'thruster_control' topic
        self.publisher_ = self.create_publisher(Int32MultiArray, 'thruster_control', 10)

        # Set the loop timer (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_thrust)

        # Initialize the ThrustAllocator
        self.thrust_allocator = ThrustAllocator()

    def publish_thrust(self):
        # Get thrust values using your ThrustAllocator class
        int_list = self.thrust_allocator.getThrustPwm([10, 0, 0])

        # Create a message and assign the data
        msg = Int32MultiArray()
        msg.data = int_list

        # Publish the message
        self.get_logger().info(f"Publishing: {int_list}")
        self.publisher_.publish(msg)


def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create the node and spin it
    thrust_publisher = ThrustPublisher()

    try:
        rclpy.spin(thrust_publisher)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    thrust_publisher.destroy_node()

    # Shutdown the ROS client library
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