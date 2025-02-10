from controls_movement.pid_controller import PIDController
from msg_types.msg import DepthIMU
from msg_types.msg import Controls
from msg_types.msg import Movement
from msg_types.msg import PIDoutputs
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor

from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
from ament_index_python.packages import get_package_share_directory
from controls_movement.param_helper import read_pid_yaml_and_generate_parameters

class VerticalPIDNode(Node):
    def __init__(self):
        super().__init__('vert_pid_node')
        package_directory = get_package_share_directory('controls_movement')
        self.declare_parameter('config_location', rclpy.Parameter.Type.STRING)
        config_location = package_directory + self.get_parameter('config_location').get_parameter_value().string_value
        self.declare_parameters(namespace='', parameters=read_pid_yaml_and_generate_parameters('vert_pid_node', config_location))

        self.wanted_depth_publisher = self.create_publisher(Float32, "/controls/wanted_depth", 10)
        # self.wanted_movement_publisher = self.create_publisher(Controls, "/controls/wanted_movement", 10)
        self.wanted_movement_publisher = self.create_publisher(Movement, "/controls/wanted_pid_movement", 10)
        self.pid_publisher = self.create_publisher(PIDoutputs, "/controls/PIDoutputs", 10)
        
        #Subscribe to Depth, RPY Data
        self.subscription = self.create_subscription(
            DepthIMU,
            '/sensors/depth_imu',
            self.drpy_callback,
            10
        )

        # Latest sensor readings
        self.current_depth = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

        # Initialise ThrustAllocator and ThrusterControl
        # self.thrustAllocator = thruster_allocator_node
        # self.thrusterControl = ThrusterControl()

        # Used for calculating dt from ros messages
        self.last_time = None


        ############################################################################
        ############################################################################
        # PID parameters

        # change back once control panel not needed
        self.desired_depth = self.get_value('desired_depth') # Example depth to maintain


        self.desired_roll = 0.0    # Example orientation targets
        self.desired_pitch = 0.0
        self.desired_yaw = 0.0
        

        # change back once control panel not needed
        self.depth_pid = PIDController(Kp=self.get_value('depth_Kp'), Ki=self.get_value('depth_Ki'), Kd=self.get_value('depth_Kd'))
        self.roll_pid = PIDController(Kp=self.get_value('roll_Kp'), Ki=self.get_value('roll_Ki'), Kd=self.get_value('roll_Kd'))
        self.pitch_pid = PIDController(Kp=self.get_value('pitch_Kp'), Ki=self.get_value('pitch_Ki'), Kd=self.get_value('pitch_Kd'))
        self.yaw_pid = PIDController(Kp=self.get_value('yaw_Kp'), Ki=self.get_value('yaw_Ki'), Kd=self.get_value('yaw_Kd'))

        ############################################################################
        ############################################################################

        self.frequency = self.get_value('PID_freq')
        self.timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(self.timer_period, self.stationkeep_callback)


    def get_value(self, param_name: str):
        return self.get_parameter(param_name).get_parameter_value().double_value

    def change_timer_period(self, new_frequency):
        self.timer.cancel()

        self.frequency = new_frequency
        self.timer_period = 1.0 / self.frequency

        self.timer = self.create_timer(self.timer_period, self.stationkeep_callback)


    def stationkeep_callback(self):
        self.change_timer_period(self.get_value('PID_freq'))

        current_time = self.get_clock().now().to_msg()
        current_seconds = current_time.sec + current_time.nanosec * 1e-9 #? sending only 60Hz why nanosec change to milli

        if self.last_time is None:
            self.last_time = current_seconds
            return #dt is still zero, so do not do PID yet
        
        dt = current_seconds - self.last_time
        self.last_time = current_seconds

        self.stationkeep(dt)



    def drpy_callback(self, msg):
        self.current_depth = msg.depth
        self.current_roll = msg.roll
        self.current_pitch = msg.pitch
        self.current_yaw = msg.yaw


    def stationkeep(self, dt):
        # change back once control panel not needed
        self.depth_pid.update_consts(new_Kp=self.get_value('depth_Kp'), new_Ki=self.get_value('depth_Ki'), new_Kd=self.get_value('depth_Kd'))
        self.desired_depth = (self.get_value('desired_depth'))
        self.roll_pid.update_consts(new_Kp=self.get_value('roll_Kp'), new_Ki=self.get_value('roll_Ki'), new_Kd=self.get_value('roll_Kd'))
        self.pitch_pid.update_consts(new_Kp=self.get_value('pitch_Kp'), new_Ki=self.get_value('pitch_Ki'), new_Kd=self.get_value('pitch_Kd'))
        # self.desired_depth = -self.get_value('desired_depth')

        depth_pid_output, dP_term, dI_term, dD_term = self.depth_pid.compute(setpoint=self.desired_depth, current_value=self.current_depth, dt=dt, kd_multiplier=self.get_value("depth_kd_multiplier"), ki_multiplier=self.get_value("depth_ki_multiplier"), integral_limit=120.0)
        translation = [0, 0, depth_pid_output - 38.0]

        roll_output, rP_term, rI_term, rD_term = self.roll_pid.compute(setpoint=self.desired_roll, current_value=self.current_roll, dt = dt, kd_multiplier=self.get_value("kd_multiplier"), ki_multiplier=self.get_value("ki_multiplier"))
        pitch_output, pP_term, pI_term, pD_term = self.pitch_pid.compute(setpoint=self.desired_pitch, current_value=self.current_pitch, dt = dt, kd_multiplier=self.get_value("kd_multiplier"), ki_multiplier=self.get_value("ki_multiplier"))
        yaw_output, yP_term, yI_term, yD_term  = self.yaw_pid.compute(setpoint=self.desired_yaw, current_value=self.current_yaw, dt = dt, kd_multiplier=self.get_value("kd_multiplier"), ki_multiplier=self.get_value("ki_multiplier"))

        rotation = [roll_output, pitch_output, yaw_output]

        # controls_msg = Controls()
        # controls_msg.translation = translation
        # self.wanted_movement_publisher.publish(controls_msg)

        # for the movement controller node
        movement_msg = Movement()
        movement_msg.x = float(translation[0])
        movement_msg.y = float(translation[1])
        movement_msg.z = float(translation[2])
        movement_msg.roll = float(rotation[0])
        movement_msg.pitch = float(rotation[1])
        movement_msg.yaw = float(rotation[2])
        self.wanted_movement_publisher.publish(movement_msg)

        # thrustAllocResult = self.thrustAllocator.getThrustPwm(translation, rotation)
        # thrustPWMs = thrustAllocResult.thrusts

        # debugging helpers
        correctDir = ("up" if self.desired_depth > self.current_depth else "down")
        outputDir = ("up" if depth_pid_output > 0 else "down")
        rotationOutput = ["+ve" if rot > 0 else "-ve" for rot in rotation]

        # self.get_logger().info(f"depthPID: {depth_pid_output},correctDir:{correctDir},outputDir:{outputDir},current_depth:{self.current_depth} desired:{self.desired_depth} integral{integral}")
        # self.get_logger().info(f"KP: {self.get_value('depth_Kp')} KD: {self.get_value('depth_Kd')} KI: {self.get_value('depth_Ki')}")
        # self.get_logger().info(f"rotationOutput:{rotationOutput},(RPY):{self.current_roll},{self.current_pitch},{self.current_yaw} int_error:{error} ")
        
        # self.get_logger().info(f"pwms: {thrustPWMs}")

        pid_msg = PIDoutputs()
        pid_msg.roll_sum = rP_term + rD_term + rI_term
        pid_msg.roll_prop = rP_term
        pid_msg.roll_deri = rD_term
        pid_msg.roll_inte = rI_term
        pid_msg.pitch_sum = pP_term + pD_term + pI_term
        pid_msg.pitch_prop = pP_term
        pid_msg.pitch_deri = pD_term
        pid_msg.pitch_inte = pI_term
        pid_msg.depth_sum = depth_pid_output
        pid_msg.depth_prop = dP_term
        pid_msg.depth_deri = dD_term
        pid_msg.depth_inte = dI_term
        self.pid_publisher.publish(pid_msg)

        msg = Float32()
        msg.data = self.desired_depth
        self.wanted_depth_publisher.publish(msg)

        # self.thrusterControl.setThrusters(thrustPWMs)

def main(args=None):
    rclpy.init(args=args)
    vert_pid_node = VerticalPIDNode()

    executor = MultiThreadedExecutor()
    executor.add_node(vert_pid_node)
    executor.spin()

    # test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()