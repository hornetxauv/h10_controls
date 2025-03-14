import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from msg_types.srv import MovementService  # Ensure correct package reference
from msg_types.msg import Movement
from custom_msgs.msg import GateDetection
from msg_types.msg import YawInfo
from msg_types.msg import DepthIMU

from controls_movement.pid_controller import PIDController

from ament_index_python.packages import get_package_share_directory
from controls_movement.param_helper import read_pid_yaml_and_generate_parameters

from std_srvs.srv import Empty


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
        self.foxglove_srv = self.create_service(Empty, 'foxglove_gate_service', self.manual_trigger_gate_start)
        
        #Subscribe to X, Z error data
        self.subscription = self.create_subscription(
            GateDetection,
            'perc/quali_gate',
            self.detection_callback,
            10
        )
        
        #Subscribe to Depth, RPY Data
        self.subscription = self.create_subscription(
            DepthIMU,
            '/sensors/depth_imu',
            self.sensors_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.publisher = self.create_publisher(Movement, "/controls/wanted_goal_movement", 10)
        self.yaw_publisher = self.create_publisher(YawInfo, "/controls/yaw_info", 10)

        self.currently_doing_gate = False
        
        self.client = self.create_client(MovementService, 'foxglove_movement_service')

        # Current errors that will be updated every time ros topic is published to
        self.x_error = 0.0
        # self.z_error = 0.0
        self.gate_sides_ratio = 1.0
        self.width = 0.0

        self.x_pid = PIDController(Kp=self.get_value('x_Kp'), Ki=self.get_value('x_Ki'), Kd=self.get_value('x_Kd'))
        # self.z_PID = PIDController(Kp=self.get_value('z_Kp'), Ki=self.get_value('z_Ki'), Kd=self.get_value('z_Kd'))
        self.sides_ratio_pid = PIDController(Kp=self.get_value('sides_ratio_Kp'), Ki=self.get_value('sides_ratio_Ki'), Kd=self.get_value('sides_ratio_Kd'))
        
        self.last_time = None

        self.current_yaw = 0
        self.last_known_gate_bearing = 0
        self.last_yaw_with_gate_detected = 0
        self.last_known_gate_size = 0
        self.last_gate_detected_seconds = 0
        self.has_reached_gate = False
        self.sustained_movement_countdown = 0

    def get_value(self, param_name: str):
        return self.get_parameter(param_name).get_parameter_value().double_value

    def detection_callback(self, msg):
        # Taking to the right to be positive dx
        self.x_error = msg.dx
        self.x_theta_error = msg.dx_theta
        # z_error = msg.dy # Note: removed due to using depth sensor
        self.gate_sides_ratio = msg.sides_ratio 
        self.width = msg.width
        self.get_logger().info(f'x_error: {self.x_error}, theta_error: {self.x_theta_error}, distance: {self.width}, gate_sides_ratio: {self.gate_sides_ratio}')

    def sensors_callback(self, msg):
        self.current_yaw = msg.yaw

    def timer_callback(self):
        self.x_pid.update_consts(new_Kp=self.get_value('x_Kp'), new_Ki=self.get_value('x_Ki'), new_Kd=self.get_value('x_Kd'))
        self.sides_ratio_pid.update_consts(new_Kp=self.get_value('sides_ratio_Kp'), new_Ki=self.get_value('sides_ratio_Ki'), new_Kd=self.get_value('sides_ratio_Kd'))

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
        y_output = 0.0
        yaw_output = 0.0

        desired_diagonal_movement = None
        crispy_toast_width = 400 # width of gate threshold

        if self.width >= crispy_toast_width:
            self.start_gate_movement()

        # only do PID if there is a gate detected, i.e. distance between gates =/= 0
        elif not self.currently_doing_gate and self.width != 0:
            # x_output, xP_term, xI_term, xD_term = self.x_pid.compute(setpoint=0.0, current_value=self.x_error, dt = dt, kd_multiplier=self.get_value("x_kd_multiplier"))
            # z_output = self.z_PID.compute(setpoint=0.0, current_value=z_error, dt = dt)
            # y_output = 1.0 # always be moving forward, this will need to change once we figure out how to determine if the gate has been passed (?)
            # yaw_output, yP_term, yI_term, yD_term  = self.sides_ratio_pid.compute(setpoint=1.0, current_value=self.gate_sides_ratio, dt = dt)

            # if abs(self.x_error) < self.get_value("x_error_threshold"):
                # y_output = self.get_value("move_forward_Kp")

            self.last_known_gate_bearing = self.x_theta_error
            self.last_yaw_with_gate_detected = self.current_yaw
            self.last_gate_detected_seconds = current_seconds
            
            # get resolved translation vectors from dx_theta
            desired_diagonal_movement = self.x_theta_error
            
        # else:
        #     # logic flow if no detect gate
        #     if self.has_reached_gate:
        #         # if moved for 10 seconds after "reaching" gate, go back to original state of not seeing the gate
        #         if current_seconds - self.last_gate_detected_seconds > 10:
        #             self.has_reached_gate = False
        #             return
        #         # else, translate towards last known gate bearing, offset by difference between current yaw and lsat yaw with gate detected
        #         desired_diagonal_movement = self.last_known_gate_bearing - (self.last_yaw_with_gate_detected - self.current_yaw)
        #     else:
        #         rotate_speed = self.get_value("rotate_speed")
        #         # rotate cockwise until find gate. need to turn off auto yaw pid in vert_pid when in this state
        #         yaw_output = rotate_speed if self.last_known_gate_bearing >= 0 else -rotate_speed
        
        if not self.currently_doing_gate:
            if desired_diagonal_movement:
                radian = np.deg2rad(desired_diagonal_movement)
                move_magnitude = self.get_value("move_forward_Kp")
                x_output = -move_magnitude * np.sin(radian)
                y_output = move_magnitude * np.cos(radian)
            else:
                x_output = 0
                y_output = 0

            self.movement_message = Movement()
            self.movement_message.x = float(x_output)
            self.movement_message.y = float(y_output)
            # self.movement_message.yaw = float(x_output)
            self.movement_message.yaw = float(yaw_output)
            self.publish()

    def manual_trigger_gate_start(self, request, response):
        self.currently_doing_gate = False
        self.start_gate_movement()
        # response.success = True
        return response

    def start_gate_movement(self):
        if not self.currently_doing_gate:
            self.get_logger().info("Started doing gate")
            self.send_request(self.moveStraightMessage())
            self.send_request(self.turn180Message())
            self.send_request(self.moveStraightMessage())
        self.currently_doing_gate = True

    def send_request(self, request):
        future = self.client.call_async(request)
        # self.get_logger().info("Requested")
        # rclpy.spin_until_future_complete(self, future)
        return future.result()

    def moveStraightMessage(self):
        move_forward_request = MovementService.Request()
        move_forward_request.duration = 2.0
        move_forward_request.movement.x = 0.0
        move_forward_request.movement.y = 10.0
        move_forward_request.movement.z = 0.0
        move_forward_request.movement.roll = 0.0
        move_forward_request.movement.pitch = 0.0
        move_forward_request.movement.yaw = 0.0
        return move_forward_request
    
    def turn180Message(self):
        turn180Message = MovementService.Request()
        turn180Message.duration = 2.0
        turn180Message.movement.x = 0.0
        turn180Message.movement.y = 0.0
        turn180Message.movement.z = 0.0
        turn180Message.movement.roll = 0.0
        turn180Message.movement.pitch = 0.0
        turn180Message.movement.yaw = 20.0
        return turn180Message

    def publish(self):
        if self.movement_message is not None:
            self.publisher.publish(self.movement_message)
            # self.get_logger().info(f"Published Movement: ${self.movement_message}")

def main(args=None):
    rclpy.init(args=args)
    pid_node = QualiGatePIDNode()

    executor = MultiThreadedExecutor()
    executor.add_node(pid_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
