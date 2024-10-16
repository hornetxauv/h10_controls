import rclpy
from rclpy.node import Node

from std_msgs.msg import String, UInt8MultiArray
from thruster_allocator import ThrustAllocator

thrustAllocator = ThrustAllocator()

class ThrustNode(Node):

    def __init__(self):
        super().__init__('thruster_node')
        
        # what format is the data being received
        self.subscription = self.create_subscription(
            String,
            'perc/imu',
            self.process_data,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(UInt8MultiArray, 'thrustPwm', 10)

    def process_data(self, msg):
        self.get_logger().info('Received data: "%s"' % msg.data)
        
        # need to change the msg.data
        pwm = thrustAllocator.getThrustPwm(msg.data, msg.data)

        thrustPwm = UInt8MultiArray()
        thrustPwm.data = pwm
        # thrustPwm.layout = '?'

        self.publisher_.publish(thrustPwm)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ThrustNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()