import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from msg_types.msg import DepthIMU  
from tf_transformations import quaternion_from_euler
import math

class DepthIMUToPoseStamped(Node):
    def __init__(self):
        super().__init__('depthimu_to_posestamped')

        self.subscription = self.create_subscription(
            DepthIMU,
            '/sensors/depth_imu',
            self.depth_imu_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseStamped,
            '/hornet/posestamped',
            10
        )

    def depth_imu_callback(self, msg):
        pose_stamped = PoseStamped()

        pose_stamped.header = Header()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'odom'
        pose_stamped.pose.position.x = 0.0
        pose_stamped.pose.position.y = 0.0
        pose_stamped.pose.position.z = msg.depth

        quaternion = quaternion_from_euler(math.radians(msg.roll), math.radians(msg.pitch), math.radians(msg.yaw))
        pose_stamped.pose.orientation.x = quaternion[0]
        pose_stamped.pose.orientation.y = quaternion[1]
        pose_stamped.pose.orientation.z = quaternion[2]
        pose_stamped.pose.orientation.w = quaternion[3]

        self.publisher.publish(pose_stamped)
        self.get_logger().info(f'depth={msg.depth}, roll={msg.roll}, pitch={msg.pitch}, yaw={msg.yaw}')



def main(args=None):
    rclpy.init(args=args)
    node = DepthIMUToPoseStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
