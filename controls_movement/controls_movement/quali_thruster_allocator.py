import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from msg_types.msg import Controls
from custom_msgs.msg import GateDetection
from controls_movement.thruster_allocator import ThrustAllocator
from ament_index_python.packages import get_package_share_directory
from controls_movement.param_helper import read_pid_yaml_and_generate_parameters
from thrusters.thrusters import ThrusterControl   #all of the lines involving ThrusterControl will not work if you have not properly installed virtual CAN

'''
Standard PID implementation
Used as a class since both yaw and pitch use different instances of the same PID algo
'''
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, setpoint, current_value, dt):
        error = setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        return output


'''
This class uses visual feed to determine desired translation in vertical and L-R directions
From which we conduct PID

e.g., on the camera feed
object is detected at (20, 25)
when the bot moves straight forward, it moves towards the point at (0, 0) (this point is fixed regardless of the robot's true orientation)
then, we will use our error values as (0, 0) - (20, 25) == (-20, -25)
on which, we will do PID
'''
class PIDNode(Node):
    def __init__(self, thruster_allocator_node):
        super().__init__('pid_node')
        package_directory = get_package_share_directory('controls_movement')
        self.declare_parameter('config_location', rclpy.Parameter.Type.STRING)
        config_location = package_directory + self.get_parameter('config_location').get_parameter_value().string_value
        self.declare_parameters(namespace='', parameters=read_pid_yaml_and_generate_parameters('pid_node', config_location))

        
        #Subscribe to X, Z error data
        self.subscription = self.create_subscription(
            GateDetection,
            'perc/quali_gate',
            self.error_callback,
            10
        )

        self.publisher = self.create_publisher(Controls, "/controls/wanted_movement", 10)

        self.movement_message = Controls()
        
        # Current errors that will be updated every time ros topic is published to
        self.x_error = 0.0
        # self.z_error = 0.0

        ############################################################################
        ############################################################################
        # PID parameters

        self.x_PID = PIDController(Kp=self.get_value('x_Kp'), Ki=self.get_value('x_Ki'), Kd=self.get_value('x_Kd'))
        # self.z_PID = PIDController(Kp=self.get_value('z_Kp'), Ki=self.get_value('z_Ki'), Kd=self.get_value('z_Kd'))

        ############################################################################
        ############################################################################
        
        
        self.thrustAllocator = thruster_allocator_node
        self.thrusterControl = ThrusterControl()

        self.last_time = None

    def get_value(self, param_name: str):
        return self.get_parameter(param_name).get_parameter_value().double_value

    def error_callback(self, msg):
        x_error = msg.dx 
        # z_error = msg.dy # Note: removed due to using depth sensor
        width = msg.width
        # self.get_logger().info(f'x_error: {x_error}, z_error: {z_error}, distance: {width}')
        self.get_logger().info(f'x_error: {x_error}, distance: {width}')

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

        # only do PID if there is a gate detected, i.e. distance between gates =/= 0
        if width != 0:
            x_output = self.x_PID.compute(setpoint=0.0, current_value=x_error, dt = dt)
            # z_output = self.z_PID.compute(setpoint=0.0, current_value=z_error, dt = dt)
            y_output = 1.0 # always be moving forward, this will need to change once we figure out how to determine if the gate has been passed (?)
        
        self.movement_message.translation = [x_output, y_output, 0]
        self.publish()

        # #attempt to set constant z_output to keep the AUV neutrally buoyant
        # z_output += self.get_value('z_offset')

        # thruster_pwm = self.thrustAllocator.getTranslationPwm([x_output, y_output, z_output])

        # # self.get_logger().info(f'x_output: {x_output}, z_output: {z_output}, y_output: {y_output}')
        # # self.get_logger().info(f'Thruster PWM Output: {thruster_pwm}')
        # self.get_logger().info(f"{self.get_value('x_Kp')} {self.get_value('x_Ki')} {self.get_value('x_Kd')}")
        
        self.thrusterControl.setThrusters(thrustValues=thruster_pwm)

    def publish(self):
        if self.movement_message is not None:
            self.publisher.publish(self.movement_message)
            self.get_logger().info(f"Published Translation: ${self.movement_message.translation} and Rotation ${self.movement_message.rotation}")


def main(args=None):
    rclpy.init(args=args)
    thruster_allocator_node = ThrustAllocator()
    pid_node = PIDNode(thruster_allocator_node)

    executor = MultiThreadedExecutor()
    executor.add_node(thruster_allocator_node)
    executor.add_node(pid_node)
    executor.spin()

    thruster_allocator_node.destroy_node()
    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
