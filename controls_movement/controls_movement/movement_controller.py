from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   #all of the lines involving ThrusterControl will not work if you have not properly installed virtual CAN
from .pid_controller import PIDController
from msg_types.msg import Controls
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
import numpy as np

class MovementControllerNode(Node):
    def __init__(self):
        super().__init__('movement_controller_node')
        
        self.subscription_depth = self.create_subscription(
            Controls,
            '/controls/wanted_depth_ori',
            self.depth_callback,
            10
        )

        self.subscription_goal = self.create_subscription(
            Controls,
            '/controls/wanted_movement',
            self.goal_callback,
            10
        )

        self.depth_translation = [0, 0, 0]
        self.depth_rotation = [0, 0, 0]
        self.goal_translation = [0, 0, 0]
        self.goal_rotation = [0, 0, 0]

        self.translation = [0, 0, 0]
        self.rotation = [0, 0, 0]

        self.thrustAllocator = ThrustAllocator()
        self.thrusterControl = ThrusterControl()
   
    def depth_callback(self, msg):
        self.get_logger().info(f"Here1")

        self.depth_translation = self.unpack_vector(msg.translation)
        self.depth_rotation= self.unpack_vector(msg.rotation)
        self.update_movements()

    def goal_callback(self, msg):
        self.get_logger().info(f"Here2")

        self.goal_translation = self.unpack_vector(msg.translation)
        self.goal_rotation = self.unpack_vector(msg.rotation)
        self.update_movements()

    def unpack_vector(self, vector):
        return np.array([vector.x, vector.y, vector.z])
    
    def update_movements(self):
        self.translation = np.add(self.depth_translation, self.goal_translation) # assuming depth and goal are independent, and goal does not contain a z factor
        self.rotation = np.add(self.depth_rotation, self.goal_rotation) # assuming goal only has a yaw
        self.get_logger().info(f"COMPUTED Translation: {self.translation} Rotation: {self.rotation}")
        thrustPWMs = self.thrustAllocator.getThrustPwm(self.translation, self.rotation)
        self.thrusterControl.setThrusters(thrustPWMs)

def main(args=None):
    rclpy.init(args=args)
    movement_controller_node = MovementControllerNode()
    try:
        while True:
            rclpy.spin(movement_controller_node)
    except KeyboardInterrupt:
        movement_controller_node.thrusterControl.killThrusters()
    finally:
        movement_controller_node.thrusterControl.killThrusters()
        movement_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()