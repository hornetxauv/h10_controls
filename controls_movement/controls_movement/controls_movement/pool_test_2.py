import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from controls_movement.thruster_allocator import ThrustAllocator

t = ThrustAllocator()


def publisher():
    # Initialize the ROS node
    rospy.init_node('int_list_publisher', anonymous=True)
    
    # Create a publisher that publishes to the 'thruster_control' topic
    pub = rospy.Publisher('thruster_control', Int32MultiArray, queue_size=10)
    
    # Set the loop rate (1 Hz in this case)
    rate = rospy.Rate(1)  # 1 Hz

    # Define the list of 7 integers
    int_list = t.getThrustPwm([10, 0, 0])

    # Prepare the message
    msg = Int32MultiArray()
    msg.data = int_list

    # Main loop to keep publishing the message
    while not rospy.is_shutdown():
        rospy.loginfo(f"Publishing: {int_list}")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

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