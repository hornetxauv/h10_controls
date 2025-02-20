from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   #all of the lines involving ThrusterControl will not work if you have not properly installed virtual CAN
from msg_types.msg import Movement
from msg_types.msg import PWMs
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.executors import MultiThreadedExecutor

class WantedMovements:
    def __init__(self, depth_translation=np.array([0, 0, 0]), depth_rotation=np.array([0, 0, 0]), goal_translation=np.array([0, 0, 0]), goal_rotation=np.array([0, 0, 0])):
        self.depth_translation = depth_translation
        self.depth_rotation = depth_rotation
        self.goal_translation = goal_translation
        self.goal_rotation = goal_rotation

    def __eq__(self, other):
        if other == None: return False
        if type(other) != WantedMovements:
            raise "Cannot compare WantedMovements with " + type(other)
        if (self.depth_translation != other.depth_translation).any(): return False
        if (self.depth_rotation != other.depth_rotation).any(): return False
        if (self.goal_translation != other.goal_translation).any(): return False
        if (self.goal_rotation != other.goal_rotation).any(): return False
        return True
    
    def __str__(self):
        return f"WantedMovements: {self.depth_translation}, {self.depth_rotation}, {self.goal_translation}, {self.goal_rotation}"

    def get_all(self):
        return (self.depth_translation, self.depth_rotation, self.goal_translation, self.goal_rotation)
    
    def copy(self):
        return WantedMovements(self.depth_translation, self.depth_rotation, self.goal_translation, self.goal_rotation)

class MovementControllerNode(Node):
    def __init__(self, thruster_allocator_node, debug=True):
        super().__init__('movement_controller_node')
        self.get_logger().info(f"movement_controller init")
        
        self.subscription_depth = self.create_subscription(
            Movement,
            '/controls/wanted_pid_movement',
            self.depth_callback,
            10
        )

        self.subscription_goal = self.create_subscription(
            Movement,
            '/controls/wanted_goal_movement',
            self.goal_callback,
            10
        )

        self.prev_wanted_movements = None
        self.curr_wanted_movements = WantedMovements()

        self.translation = [0, 0, 0]
        self.rotation = [0, 0, 0]

        self.thrustAllocator = thruster_allocator_node
        self.thrusterControl = ThrusterControl()

        self.update_movements_timer = self.create_timer(0.1, self.update_movements)

        # FOXGLOVE DEBUGGING
        self.debug = debug
        self.PWMs_publisher = self.create_publisher(PWMs, "/controls/PWMs", 10)
        self.full_movement_publisher = self.create_publisher(Movement, "/controls/full_movement", 10)

    def depth_callback(self, msg):
        # self.get_logger().info(f"Depth callback triggered")
        # self.get_logger().info(f"READ Translation: {msg.x} Rotation: {msg.z}")
        depth_translation, depth_rotation = self.unpack_vector(msg)
        self.curr_wanted_movements.depth_translation = depth_translation
        self.curr_wanted_movements.depth_rotation = depth_rotation

    def goal_callback(self, msg):
        # self.get_logger().info(f"Goal callback triggered")
        # self.get_logger().info(f"READ Translation: {self.goal_translation} Rotation: {self.goal_rotation}")
        goal_translation, goal_rotation = self.unpack_vector(msg)
        self.curr_wanted_movements.goal_translation = goal_translation
        self.curr_wanted_movements.goal_rotation = goal_rotation

    def unpack_vector(self, vector):
        return np.array([vector.x, vector.y, vector.z]), np.array([vector.roll, vector.pitch, -vector.yaw])
    
    def update_movements(self):
        # self.get_logger().info(str(self.prev_wanted_movements))
        # self.get_logger().info(str(self.curr_wanted_movements))
        if self.prev_wanted_movements == self.curr_wanted_movements:
            self.get_logger().info("Avoided unnecessary vector calc")
            return
        self.prev_wanted_movements = self.curr_wanted_movements.copy()
        depth_translation, depth_rotation, goal_translation, goal_rotation = self.curr_wanted_movements.get_all()
        self.translation = np.add(depth_translation, goal_translation) # assuming depth and goal are independent, and goal does not contain a z factor
        self.rotation = np.add(depth_rotation, goal_rotation) # assuming goal only has a yaw
        # self.get_logger().info(f"COMPUTED Translation: {self.translation} Rotation: {self.rotation}")
        thrustAllocResult = self.thrustAllocator.getThrustPwm(self.translation, self.rotation)
        thrustPWMs = thrustAllocResult.thrusts
        self.publish_thrusters(thrustPWMs, thrustAllocResult)

        movement_msg = Movement()
        movement_msg.x = float(self.translation[0])
        movement_msg.y = float(self.translation[1])
        movement_msg.z = float(self.translation[2])
        movement_msg.roll = float(self.rotation[0])
        movement_msg.pitch = float(self.rotation[1])
        movement_msg.yaw = float(self.rotation[2])
        self.full_movement_publisher.publish(movement_msg)

    def publish_thrusters(self, thrustPWMs, thrustAllocResult):
        PWMs_msg = PWMs()
        PWMs_msg.math_eqn_solvable = thrustAllocResult.solveSuccess
        PWMs_msg.one = int(thrustPWMs[0])
        PWMs_msg.two = int(thrustPWMs[1])
        PWMs_msg.three = int(thrustPWMs[2])
        PWMs_msg.four = int(thrustPWMs[3])
        PWMs_msg.five = int(thrustPWMs[4])
        PWMs_msg.six = int(thrustPWMs[5])
        PWMs_msg.seven = int(thrustPWMs[6])
        self.PWMs_publisher.publish(PWMs_msg)

def main(args=None):
    # rclpy.init(args=args)
    # movement_controller_node = MovementControllerNode(True)
    # try:
    #     while True:
    #         rclpy.spin(movement_controller_node)
    # except KeyboardInterrupt:
    #     movement_controller_node.thrusterControl.killThrusters()
    # finally:
    #     movement_controller_node.thrusterControl.killThrusters()
    #     movement_controller_node.destroy_node()
    #     rclpy.shutdown()

    rclpy.init(args=args)
    thruster_allocator_node = ThrustAllocator()
    vert_pid_node = MovementControllerNode(thruster_allocator_node)

    executor = MultiThreadedExecutor()
    executor.add_node(thruster_allocator_node)
    executor.add_node(vert_pid_node)
    executor.spin()

    thruster_allocator_node.destroy_node()
    # test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()