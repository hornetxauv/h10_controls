import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Float32
from msg_types.msg import IMU


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')   #this is the name of the node
        self.subscription = self.create_subscription(
            Float32,
            '/sensors/depth',
            self.depth_callback,
            10
        )
        # #Subscribe to Roll, Pitch, Yaw data
        # self.subscriptionIMU = self.create_subscription(
        #     IMU, #custom IMU msg type
            # '/sensors/imu',  
        #     self.publish_callback,
        #     10
        # )
        self.publisher_ = self.create_publisher(Float32, 'sensor_topic', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.publish_callback)
        self.i = 0

    def depth_callback(self, msg):
        # msg = Float32()
        # msg.data = 20.25
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
