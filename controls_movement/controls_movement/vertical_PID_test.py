from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   #all of the lines involving ThrusterControl will not work if you have not properly installed virtual CAN
from controls_movement.pid_controller import PIDController
from msg_types.msg import DepthIMU
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
from control_panel.control_panel import create_control_panel, ControlPanelItem as CPI #this is a package in PL repo

values = {
    'depth_kd_multiplier': CPI(value=1, maximum=5, minimum=0, multiplier=1),
    'depth_ki_multiplier': CPI(value=1, maximum=5, minimum=0, multiplier=1),
    'ki_multiplier': CPI(value=3, maximum=5, minimum=0, multiplier=1),
    'kd_multiplier': CPI(value=1, maximum=5, minimum=0, multiplier=1),
    'desired_depth': CPI(value=1.0, maximum=200, minimum=0, multiplier=1),
    'depth Kp': CPI(value=0.05, maximum=50, minimum=0, multiplier=10),
    'depth Ki': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'depth Kd': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'roll Kp': CPI(value=0.03, maximum=1, minimum=0, multiplier=100),
    'roll Ki': CPI(value=0.11, maximum=1, minimum=0, multiplier=100),
    'roll Kd': CPI(value=0.15, maximum=1, minimum=0, multiplier=100),
    'pitch Kp': CPI(value=0.03, maximum=1, minimum=0, multiplier=100),
    'pitch Ki': CPI(value=0.11, maximum=1, minimum=0, multiplier=100),
    'pitch Kd': CPI(value=0.15, maximum=1, minimum=0, multiplier=100),
    'yaw Kp': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'yaw Ki': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'yaw Kd': CPI(value=0, maximum=1, minimum=0, multiplier=100),
}

create_control_panel("verti PID", values)

class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')
        
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
        self.desired_depth = -values['desired_depth'].value # Example depth to maintain


        self.desired_roll = 0.0    # Example orientation targets
        self.desired_pitch = 0.0
        self.desired_yaw = 0.0


        # self.depth_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        # self.roll_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        # self.pitch_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        # self.yaw_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        

        # change back once control panel not needed
        self.depth_pid = PIDController(Kp=values['depth Kp'].value, Ki=values['depth Ki'].value, Kd=values['depth Kd'].value)
        self.roll_pid = PIDController(Kp=values['roll Kp'].value, Ki=values['roll Ki'].value, Kd=values['roll Kd'].value)
        self.pitch_pid = PIDController(Kp=values['pitch Kp'].value, Ki=values['pitch Ki'].value, Kd=values['pitch Kd'].value)
        self.yaw_pid = PIDController(Kp=values['yaw Kp'].value, Ki=values['yaw Ki'].value, Kd=values['yaw Kd'].value)

        ############################################################################
        ############################################################################


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
        self.depth_pid.update_consts(new_Kp=values['depth Kp'].value, new_Ki=values['depth Ki'].value, new_Kd=values['depth Kd'].value)
        self.desired_depth = -(values['desired_depth'].value)/100
        self.roll_pid.update_consts(new_Kp=values['roll Kp'].value, new_Ki=values['roll Ki'].value, new_Kd=values['roll Kd'].value)
        self.pitch_pid.update_consts(new_Kp=values['pitch Kp'].value, new_Ki=values['pitch Ki'].value, new_Kd=values['pitch Kd'].value)
        # self.desired_depth = -values['desired_depth'].value

        depth_pid_output = self.depth_pid.compute(setpoint=self.desired_depth, current_value=self.current_depth, dt=dt, kd_multiplier=values["depth_kd_multiplier"].value, ki_multiplier=values["depth_ki_multiplier"].value)[0]
        translation = [0, 0, -100*depth_pid_output]

        roll_output, error, kd, deri = self.roll_pid.compute(setpoint=self.desired_roll, current_value=self.current_roll, dt = dt, kd_multiplier=values["kd_multiplier"].value, ki_multiplier=values["ki_multiplier"].value)
        pitch_output = self.pitch_pid.compute(setpoint=self.desired_pitch, current_value=self.current_pitch, dt = dt, kd_multiplier=values["kd_multiplier"].value, ki_multiplier=values["ki_multiplier"].value)[0]
        yaw_output = self.yaw_pid.compute(setpoint=self.desired_yaw, current_value=self.current_yaw, dt = dt, kd_multiplier=values["kd_multiplier"].value, ki_multiplier=values["ki_multiplier"].value)[0]

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