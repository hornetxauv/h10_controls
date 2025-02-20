import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory
from controls_movement.param_helper import read_pid_yaml_and_generate_parameters
from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   
from msg_types.msg import Movement


'''
[X, Y, Z]
- +X is right
- +Y is forward
- +Z is up

[R, P, Y]
 - +R is roll right
 - +P is pitch up
 - +Y is yaw left
'''
class DirectionTestNode(Node):
    def __init__(self):
        super().__init__('direction_test_node')
        package_directory = get_package_share_directory('controls_movement')
        self.declare_parameter('config_location', rclpy.Parameter.Type.STRING)
        config_location = package_directory + self.get_parameter('config_location').get_parameter_value().string_value
        self.declare_parameters(namespace='', parameters=read_pid_yaml_and_generate_parameters('direction_test_node', config_location))
        self.x = 200
        self.goal_publisher = self.create_publisher(Movement, "/controls/wanted_goal_movement", 10)
        
        self.timer = self.create_timer(1.0, self.callback) #replaced with timer because while loop causes threading issues

    def callback(self):
        self.get_logger().info(f"integrated dir test callback")
        # Translation = [0, 0, 0]
        movement_msg = Movement()
        movement_msg.x = float(self.get_value("x")-self.x/2)
        movement_msg.y = float(self.get_value("y")-self.x/2)
        movement_msg.z = float(self.get_value("z")-self.x/2)
        movement_msg.roll = float(self.get_value("roll")-self.x/2)
        movement_msg.pitch = float(self.get_value("pitch")-self.x/2)
        movement_msg.yaw = float(self.get_value("yaw")-self.x/2)
        # Translation = [self.get_value("x")-self.x/2, self.get_value("y")-self.x/2, self.get_value("z")-self.x/2]

        #Rotation = [0, 0, 0]
        #if all(val == 0 for val in Translation):

        # thruster_pwm = self.thrusterAllocator.getTranslationPwm(Translation).thrusts

        # self.get_logger().info(f"{thruster_pwm}")

        # self.thrusterController.setThrusters(thrustValues=thruster_pwm)
        self.get_logger().info(f"{movement_msg}")
        self.goal_publisher.publish(movement_msg)

    def get_value(self, param_name: str):
            return self.get_parameter(param_name).get_parameter_value().double_value

def main(args=None):
    rclpy.init(args=args)
    # thruster_allocator_node = ThrustAllocator()
    # thruster_controller = ThrusterControl()
    direction_test_node = DirectionTestNode()

    executor = MultiThreadedExecutor()
    executor.add_node(direction_test_node)
    executor.spin()
    direction_test_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()