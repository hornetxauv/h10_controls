from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   #all of the lines involving ThrusterControl will not work if you have not properly installed virtual CAN
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
    'x': CPI(value=20, maximum=40, minimum=0, multiplier=1),
    'y': CPI(value=20, maximum=40, minimum=0, multiplier=1),
    'z': CPI(value=20, maximum=40, minimum=0, multiplier=1),
    'depth Kp': CPI(value=0, maximum=10, minimum=0.1, multiplier=10),
    'depth Ki': CPI(value=0, maximum=1, minimum=0.01, multiplier=100),
    'depth Kd': CPI(value=0, maximum=10, minimum=0.1, multiplier=10),
    'roll Kp': CPI(value=0, maximum=10, minimum=0.1, multiplier=10),
    'roll Ki': CPI(value=0, maximum=1, minimum=0.01, multiplier=100),
    'roll Kd': CPI(value=0, maximum=10, minimum=0.1, multiplier=10),
    'pitch Kp': CPI(value=0, maximum=10, minimum=0.1, multiplier=10),
    'pitch Ki': CPI(value=0, maximum=1, minimum=0.01, multiplier=100),
    'pitch Kd': CPI(value=0, maximum=10, minimum=0.1, multiplier=10),
    'yaw Kp': CPI(value=0, maximum=10, minimum=0.1, multiplier=10),
    'yaw Ki': CPI(value=0, maximum=1, minimum=0.01, multiplier=100),
    'yaw Kd': CPI(value=0, maximum=10, minimum=0.1, multiplier=10),
}
# can't seem to use simultaneously with thruster biases control panel... sometimes. idk.
create_control_panel("verti PID", values)

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


        self.desired_depth = 10.0  # Example depth to maintain
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
        self.ori_freq = 0.05


        ############################################################################
        ############################################################################



        self.thrustAllocator = ThrustAllocator()
        self.thrusterControl = ThrusterControl()   
        self.last_time = None

        # Timer for orientation control 
        '''
        we will continuously control for depth
        and every self.ori_freq seconds, we will make a correction for orientation
        '''
        self.orientation_timer = self.create_timer(self.ori_freq, self.control_orientation) #! to uncomment

    def imu_callback(self, msg):
        self.current_roll = msg.roll
        self.current_pitch = msg.pitch
        self.current_yaw = msg.yaw
        # (past) yaw is depth for now bcos joel made a mistake
        # self.current_depth = msg.yaw
    
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
        pid_output = self.depth_pid.compute(setpoint=self.desired_depth, current_value=self.current_depth, dt=dt)
        translation = [0, 0, pid_output]
        # translation = [values["x"].value-20, values["y"].value-20, values["z"].value-20]
        thruster_pwm = self.thrustAllocator.getTranslationPwm(translation)
        # thruster_pwm= [
        #     values["0 pwm"].value, 
        #     values["1 pwm"].value, 
        #     values["2 pwm"].value, 
        #     values["3 pwm"].value, 
        #     values["4 pwm"].value, 
        #     values["5 pwm"].value, 
        #     values["6 pwm"].value]

        self.get_logger().info(f'Depth: {self.current_depth} Thruster PWM Output: {thruster_pwm} dt: {dt}')

        # set thruster values to the computed pwm values from ThrustAllocator
        self.thrusterControl.setThrusters(thrustValues=thruster_pwm) #! to change back
        # self.thrusterControl.setThrusters(thrustValues=[140, 140, 140, 140, 140, 140, 140])

    def control_orientation(self):
        print("control orientation called")
        roll_output = self.roll_pid.compute(setpoint=self.desired_roll, current_value=self.current_roll, dt = self.ori_freq)
        pitch_output = self.pitch_pid.compute(setpoint=self.desired_pitch, current_value=self.current_pitch, dt = self.ori_freq)
        yaw_output = self.yaw_pid.compute(setpoint=self.desired_yaw, current_value=self.current_yaw, dt = self.ori_freq)

        thruster_pwm = self.thrustAllocator.getRotationPwm([roll_output, pitch_output, yaw_output])

        self.get_logger().info(f'Curr Depth: {self.current_depth} Thruster PWM: {thruster_pwm} (R, P, Y) = ({self.current_roll}, {self.current_pitch}, {self.current_yaw})')

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