from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   #all of the lines involving ThrusterControl will not work if you have not properly installed virtual CAN
from .pid_controller import PIDController
from msg_types.msg import IMU
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
from control_panel.control_panel import create_control_panel, ControlPanelItem as CPI #this is a package in PL repo

values = {
    # 'z offset': CPI(value=1.0, maximum=50, minimum=0.1, multiplier=10),
    # '0 pwm': CPI(value=140, maximum=250, minimum=0, multiplier=1),
    # '1 pwm': CPI(value=140, maximum=250, minimum=0, multiplier=1),
    # '2 pwm': CPI(value=140, maximum=250, minimum=0, multiplier=1),
    # '3 pwm': CPI(value=140, maximum=250, minimum=0, multiplier=1),
    # '4 pwm': CPI(value=140, maximum=250, minimum=0, multiplier=1),
    # '5 pwm': CPI(value=140, maximum=250, minimum=0, multiplier=1),
    # '6 pwm': CPI(value=140, maximum=250, minimum=0, multiplier=1),
    'multiplier': CPI(value=1, maximum=5, minimum=0, multiplier=1),
    'desired_depth': CPI(value=1.0, maximum=2, minimum=0, multiplier=10),
    'depth Kp': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'depth Ki': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'depth Kd': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'roll Kp': CPI(value=0.05, maximum=1, minimum=0, multiplier=100),
    'roll Ki': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'roll Kd': CPI(value=0.1, maximum=1, minimum=0, multiplier=100),
    'pitch Kp': CPI(value=0.05, maximum=1, minimum=0, multiplier=100),
    'pitch Ki': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'pitch Kd': CPI(value=0.1, maximum=1, minimum=0, multiplier=100),
    'yaw Kp': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'yaw Ki': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'yaw Kd': CPI(value=0, maximum=1, minimum=0, multiplier=100),
}
# can't seem to use simultaneously with thruster biases control panel... sometimes. idk.
create_control_panel("verti PID", values)

class PIDNode(Node):
    def __init__(self):
        super().__init__('pid_node')
        
        #Subscribe to Depth Data
        self.subscription = self.create_subscription(
            Float32,
            '/sensors/depth',
            self.depth_callback,
            10
        )

        #Subscribe to Roll, Pitch, Yaw data
        self.subscriptionIMU = self.create_subscription(
            IMU, #custom IMU msg type
            '/sensors/imu',  
            self.imu_callback,
            10
        )

        # Latest sensor readings
        self.current_depth = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0


        ############################################################################
        ############################################################################
        # PID parameters


        self.desired_depth = -values['desired_depth'].value # Example depth to maintain
        self.desired_roll = 0.0    # Example orientation targets
        self.desired_pitch = 0.0
        self.desired_yaw = 0.0


        # self.depth_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)

        # self.roll_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        # self.pitch_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        # self.yaw_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        
        self.depth_pid = PIDController(Kp=values['depth Kp'].value, Ki=values['depth Ki'].value, Kd=values['depth Kd'].value)

        self.roll_pid = PIDController(Kp=values['roll Kp'].value, Ki=values['roll Ki'].value, Kd=values['roll Kd'].value)
        self.pitch_pid = PIDController(Kp=values['pitch Kp'].value, Ki=values['pitch Ki'].value, Kd=values['pitch Kd'].value)
        self.yaw_pid = PIDController(Kp=values['yaw Kp'].value, Ki=values['yaw Ki'].value, Kd=values['yaw Kd'].value)

        # Time passed between each orientation correction (in seconds)
        self.ori_freq = 0.01


        ############################################################################
        ############################################################################



        self.thrustAllocator = ThrustAllocator()
        self.thrusterControl = ThrusterControl()   
        self.last_time = None
        self.ori_last_time = None

        # Timer for orientation control 
        '''
        we will continuously control for depth
        and every self.ori_freq seconds, we will make a correction for orientation
        '''
        # self.orientation_timer = self.create_timer(self.ori_freq, self.control_orientation) #! to uncomment

    def imu_callback(self, msg):
        self.current_roll = msg.roll
        self.current_pitch = msg.pitch
        self.current_yaw = msg.yaw

        current_time = self.get_clock().now().to_msg()
        current_seconds = current_time.sec + current_time.nanosec * 1e-9 #? sending only 60Hz why nanosec change to milli


        if self.ori_last_time is None:
            self.ori_last_time = current_seconds
            return #dt is still zero, so do not do PID yet
        
        dt = current_seconds - self.ori_last_time
        self.ori_last_time = current_seconds

        self.control_orientation(dt)
    
    def depth_callback(self, msg):
        self.current_depth = msg.data

        # self.get_logger().info('Depth: "%s"' % self.current_depth)

        '''
        dt needs to be calculated dynamically, as the duration between each publish to the imu ros topic may vary
        the following code uses the timestamp that comes with each ros msg to calculate dt
        '''
        # Extract the timestamp from the message header
        current_time = self.get_clock().now().to_msg()
        current_seconds = current_time.sec + current_time.nanosec * 1e-9 #? sending only 60Hz why nanosec change to milli

        if self.last_time is None:
            self.last_time = current_seconds
            return #dt is still zero, so do not do PID yet
        
        dt = current_seconds - self.last_time
        self.last_time = current_seconds

        self.control_depth(dt)

    def control_depth(self, dt):
        # Compute PID output
        print("control depth called")
        
        #these lines are needed because of control panel, once control panel no need, these can be removed
        self.depth_pid.update_consts(new_Kp=values['depth Kp'].value, new_Ki=values['depth Ki'].value, new_Kd=values['depth Kd'].value)
        self.desired_depth = -values['desired_depth'].value

        pid_output = self.depth_pid.compute(setpoint=self.desired_depth, current_value=self.current_depth, dt=dt, multiplier=values["multiplier"].value)[0]
        translation = [0, 0, pid_output]
        thruster_pwm = self.thrustAllocator.getTranslationPwm(translation)

        # self.get_logger().info(f'Depth: {self.current_depth} Thruster PWM Output: {thruster_pwm} dt: {dt}')

        # set thruster values to the computed pwm values from ThrustAllocator
        # self.thrusterControl.setThrusters(thrustValues=thruster_pwm) #! to change back

        # self.thrusterControl.setThrusters(thrustValues=[140, 140, 140, 140, 140, 140, 140])

    def control_orientation(self, dt):
        print("control orientation called")

        #these lines are needed because of control panel, once control panel no need, these can be removed
        self.roll_pid.update_consts(new_Kp=values['roll Kp'].value, new_Ki=values['roll Ki'].value, new_Kd=values['roll Kd'].value)
        # self.roll_pid.update_consts(new_Kp=values['roll Kp'].value, new_Ki=values['roll Ki'].value, new_Kd=values['roll Kd'].value)
        # self.get_logger().info(self.pitch_pid.update_consts(new_Kp=values['pitch Kp'].value, new_Ki=values['pitch Ki'].value, new_Kd=values['pitch Kd'].value))
        self.pitch_pid.update_consts(new_Kp=values['pitch Kp'].value, new_Ki=values['pitch Ki'].value, new_Kd=values['pitch Kd'].value)
        self.desired_depth = -values['desired_depth'].value

        roll_output = self.roll_pid.compute(setpoint=self.desired_roll, current_value=self.current_roll, dt = dt, multiplier=values["multiplier"].value)[0]
        pitch_output, error, kd, deri = self.pitch_pid.compute(setpoint=self.desired_pitch, current_value=self.current_pitch, dt = dt, multiplier=values["multiplier"].value)
        # self.get_logger().info(f'pitch output: {pitch_output}, error*Kp: {error}, kd: {kd}, deri: {deri}, dt: {dt}')
        self.get_logger().info(f'pitch output: {round(pitch_output, 4)}, error: {round(error, 4)} kd: {kd} deri: {round(deri, 4)}')
        yaw_output = self.yaw_pid.compute(setpoint=self.desired_yaw, current_value=self.current_yaw, dt = dt, multiplier=values["multiplier"].value)[0]

        thruster_pwm = self.thrustAllocator.getRotationPwm([roll_output, pitch_output, yaw_output])
# PID: {self.roll_pid} 
        # self.get_logger().info(f'Curr Depth: {self.current_depth} Thruster PWM: {thruster_pwm} (R, P, Y) = ({self.current_roll}, {self.current_pitch}, {self.current_yaw}) roll_output: {roll_output} pitch_output: {pitch_output}')
        # self.get_logger().info(f'pitch_output: {pitch_output}, pitch error*Kp: {error}, pitch deri: {deri}')

        self.thrusterControl.setThrusters(thrustValues=thruster_pwm)

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