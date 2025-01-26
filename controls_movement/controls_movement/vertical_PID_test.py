from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   #all of the lines involving ThrusterControl will not work if you have not properly installed virtual CAN
from controls_movement.pid_controller import PIDController
from msg_types.msg import DepthIMU
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
from ament_index_python.packages import get_package_share_directory
from controls_movement.param_helper import read_pid_yaml_and_generate_parameters

class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')
        package_directory = get_package_share_directory('controls_movement')
        self.declare_parameter('config_location', rclpy.Parameter.Type.STRING)
        config_location = package_directory + self.get_parameter('config_location').get_parameter_value().string_value
        self.declare_parameters(namespace='', parameters=read_pid_yaml_and_generate_parameters('pid_node', config_location))

        
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
        self.thrustAllocator = ThrustAllocator()
        self.thrusterControl = ThrusterControl()   

        # Used for calculating dt from ros messages
        self.last_time = None


        ############################################################################
        ############################################################################
        # PID parameters

        # change back once control panel not needed
        self.desired_depth = -get_value('desired_depth') # Example depth to maintain


        self.desired_roll = 0.0    # Example orientation targets
        self.desired_pitch = 0.0
        self.desired_yaw = 0.0


        # self.depth_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        # self.roll_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        # self.pitch_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        # self.yaw_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        

        # change back once control panel not needed
        self.depth_pid = PIDController(Kp=get_value('depth_Kp'), Ki=get_value('depth_Ki'), Kd=get_value('depth_Kd'))
        self.roll_pid = PIDController(Kp=get_value('roll_Kp'), Ki=get_value('roll_Ki'), Kd=get_value('roll_Kd'))
        self.pitch_pid = PIDController(Kp=get_value('pitch_Kp'), Ki=get_value('pitch_Ki'), Kd=get_value('pitch_Kd'))
        self.yaw_pid = PIDController(Kp=get_value('yaw_Kp'), Ki=get_value('yaw_Ki'), Kd=get_value('yaw_Kd'))

        ############################################################################
        ############################################################################

    def get_value(self, param_name: str):
        return self.get_parameter(param_name).get_parameter_value().double_value


    def drpy_callback(self, msg):
        self.current_depth = -msg.depth
        self.current_roll = msg.roll
        self.current_pitch = msg.pitch
        self.current_yaw = msg.yaw

        current_time = self.get_clock().now().to_msg()
        current_seconds = current_time.sec + current_time.nanosec * 1e-9 #? sending only 60Hz why nanosec change to milli


        if self.last_time is None:
            self.last_time = current_seconds
            return #dt is still zero, so do not do PID yet
        
        dt = current_seconds - self.last_time
        self.last_time = current_seconds

        self.stationkeep(dt)


    def stationkeep(self, dt):
        # change back once control panel not needed
        self.depth_pid.update_consts(new_Kp=get_value('depth_Kp'), new_Ki=get_value('depth_Ki'), new_Kd=get_value('depth_Kd'))
        self.desired_depth = -(get_value('desired_depth'))/100
        self.roll_pid.update_consts(new_Kp=get_value('roll_Kp'), new_Ki=get_value('roll_Ki'), new_Kd=get_value('roll_Kd'))
        self.pitch_pid.update_consts(new_Kp=get_value('pitch_Kp'), new_Ki=get_value('pitch_Ki'), new_Kd=get_value('pitch_Kd'))
        # self.desired_depth = -get_value('desired_depth')

        depth_pid_output = self.depth_pid.compute(setpoint=self.desired_depth, current_value=self.current_depth, dt=dt, kd_multiplier=get_value("depth_kd_multiplier"), ki_multiplier=get_value("depth_ki_multiplier"))[0]
        translation = [0, 0, -100*depth_pid_output]

        roll_output, error, kd, deri = self.roll_pid.compute(setpoint=self.desired_roll, current_value=self.current_roll, dt = dt, kd_multiplier=get_value("kd_multiplier"), ki_multiplier=get_value("ki_multiplier"))
        pitch_output = self.pitch_pid.compute(setpoint=self.desired_pitch, current_value=self.current_pitch, dt = dt, kd_multiplier=get_value("kd_multiplier"), ki_multiplier=get_value("ki_multiplier"))[0]
        yaw_output = self.yaw_pid.compute(setpoint=self.desired_yaw, current_value=self.current_yaw, dt = dt, kd_multiplier=get_value("kd_multiplier"), ki_multiplier=get_value("ki_multiplier"))[0]

        rotation = [roll_output, pitch_output, yaw_output]

        thrustPWMs = self.thrustAllocator.getThrustPwm(translation, rotation)

        # debugging helpers
        correctDir = ("up" if self.desired_depth > self.current_depth else "down")
        outputDir = ("up" if depth_pid_output > 0 else "down")
        rotationOutput = ["+ve" if rot > 0 else "-ve" for rot in rotation]

        self.get_logger().info(f"depthPID: {depth_pid_output},correctDir:{correctDir},outputDir:{outputDir},current_depth:{self.current_depth} desired:{self.desired_depth} error{error}")
        # self.get_logger().info(f"rotationOutput:{rotationOutput},(RPY):{self.current_roll},{self.current_pitch},{self.current_yaw} int_error:{error} ")



        self.thrusterControl.setThrusters(thrustPWMs)


def main(args=None):
    rclpy.init(args=args)
    pid_node = PIDNode()
    try:
        while True:
            rclpy.spin(pid_node)
    except KeyboardInterrupt:
        pid_node.thrusterControl.killThrusters()
    finally:
        pid_node.thrusterControl.killThrusters()
        pid_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()