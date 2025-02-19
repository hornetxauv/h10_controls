import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from msg_types.msg import Movement
from custom_msgs.msg import GateDetection
from msg_types.msg import YawInfo

from controls_movement.pid_controller import PIDController

from ament_index_python.packages import get_package_share_directory
from controls_movement.param_helper import read_pid_yaml_and_generate_parameters


'''
This class uses visual feed to determine desired translation in vertical and L-R directions
From which we conduct PID

e.g., on the camera feed
object is detected at (20, 25)
when the bot moves straight forward, it moves towards the point at (0, 0) (this point is fixed regardless of the robot's true orientation)
then, we will use our error values as (0, 0) - (20, 25) == (-20, -25)
on which, we will do PID
'''
class QualiGatePIDNode(Node):
    def __init__(self):
        super().__init__('quali_gate_pid_node')
        package_directory = get_package_share_directory('controls_movement')
        self.declare_parameter('config_location', rclpy.Parameter.Type.STRING)
        config_location = package_directory + self.get_parameter('config_location').get_parameter_value().string_value
        self.declare_parameters(namespace='', parameters=read_pid_yaml_and_generate_parameters('quali_gate_pid_node', config_location))

        
        #Subscribe to X, Z error data
        self.subscription = self.create_subscription(
            GateDetection,
            'perc/quali_gate',
            self.detection_callback,
            10
        )

        self.publisher = self.create_publisher(Movement, "/controls/wanted_goal_movement", 10)
        self.yaw_publisher = self.create_publisher(YawInfo, "/controls/yaw_info", 10)
        
        # Current errors that will be updated every time ros topic is published to
        self.x_error = 0.0
        # self.z_error = 0.0
        self.gate_sides_ratio = 1.0

        self.x_pid = PIDController(Kp=self.get_value('x_Kp'), Ki=self.get_value('x_Ki'), Kd=self.get_value('x_Kd'))
        # self.z_PID = PIDController(Kp=self.get_value('z_Kp'), Ki=self.get_value('z_Ki'), Kd=self.get_value('z_Kd'))
        self.sides_ratio_pid = PIDController(Kp=self.get_value('sides_ratio_Kp'), Ki=self.get_value('sides_ratio_Ki'), Kd=self.get_value('sides_ratio_Kd'))
        
        self.last_time = None

    def get_value(self, param_name: str):
        return self.get_parameter(param_name).get_parameter_value().double_value

    def detection_callback(self, msg):
        self.x_pid.update_consts(new_Kp=self.get_value('x_Kp'), new_Ki=self.get_value('x_Ki'), new_Kd=self.get_value('x_Kd'))
        self.sides_ratio_pid.update_consts(new_Kp=self.get_value('sides_ratio_Kp'), new_Ki=self.get_value('sides_ratio_Ki'), new_Kd=self.get_value('sides_ratio_Kd'))

        # Taking to the right to be positive dx
        self.x_error = msg.dx 
        # z_error = msg.dy # Note: removed due to using depth sensor
        self.gate_sides_ratio = msg.sides_ratio 
        width = msg.width
        self.get_logger().info(f'x_error: {self.x_error}, distance: {width}, gate_sides_ratio: {self.gate_sides_ratio}')

        # Extract the timestamp from the message header
        current_time = self.get_clock().now().to_msg()
        current_seconds = current_time.sec + current_time.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_seconds
            return #dt is still zero, so do not do PID yet

        dt = current_seconds - self.last_time
        self.last_time = current_seconds

        x_output = 0.0
        # z_output = 0.0
        # y_output = 0.0
        yaw_output = 0.0

        # only do PID if there is a gate detected, i.e. distance between gates =/= 0
        if width != 0:
            x_output, xP_term, xI_term, xD_term = self.x_pid.compute(setpoint=0.0, current_value=self.x_error, dt = dt, kd_multiplier=self.get_value("x_kd_multiplier"))
            # z_output = self.z_PID.compute(setpoint=0.0, current_value=z_error, dt = dt)
            # y_output = 1.0 # always be moving forward, this will need to change once we figure out how to determine if the gate has been passed (?)
            # yaw_output, yP_term, yI_term, yD_term  = self.sides_ratio_pid.compute(setpoint=1.0, current_value=self.gate_sides_ratio, dt = dt)

        self.movement_message = Movement()
        # self.movement_message.x = float(x_output)
        # self.movement_message.yaw = float(yaw_output)
        self.movement_message.yaw = float(x_output)
        self.publish()

    def publish(self):
        if self.movement_message is not None:
            self.publisher.publish(self.movement_message)
            self.get_logger().info(f"Published Movement: ${self.movement_message}")

def main(args=None):
    rclpy.init(args=args)
    pid_node = QualiGatePIDNode()

    executor = MultiThreadedExecutor()
    executor.add_node(pid_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
