from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   #all of the lines involving ThrusterControl will not work if you have not properly installed virtual CAN
from msg_types.msg import IMU
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32

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


        self.depth_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)

        self.roll_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        self.pitch_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        self.yaw_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)


        # Time passed between each orientation correction (in seconds)
        self.ori_freq = 2.0


        ############################################################################
        ############################################################################



        self.thrustAllocator = ThrustAllocator()
        #self.thrusterControl = ThrusterControl()   
        self.last_time = None

        # Timer for orientation control 
        '''
        we will continuously control for depth
        and every 5 seconds, we will make a correction for orientation
        '''
        self.orientation_timer = self.create_timer(self.ori_freq, self.control_orientation)

    def imu_callback(self, msg):
        self.current_roll = msg.roll
        self.current_pitch = msg.pitch
        self.current_yaw = msg.yaw
    
    def depth_callback(self, msg):
        self.current_depth = msg.data

        '''
        dt needs to be calculated dynamically, as the duration between each publish to the imu ros topic may vary
        the following code uses the timestamp that comes with each ros msg to calculate dt
        '''
        # Extract the timestamp from the message header
        current_time = self.get_clock().now().to_msg()
        current_seconds = current_time.sec + current_time.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_seconds
            return #dt is still zero, so do not do PID yet
        
        dt = current_seconds - self.last_time
        self.last_time = current_seconds

        self.control_depth(dt)

    def control_depth(self, dt):
        # Compute PID output
        pid_output = self.depth_pid.compute(setpoint=self.desired_depth, current_value=self.current_depth, dt=dt)
        thruster_pwm = self.thrustAllocator.getTranslationPwm([0, 0, pid_output])

        self.get_logger().info(f'Thruster PWM Output: {thruster_pwm}')
        self.get_logger().info(f'dt: {dt}')

        # set thruster values to the computed pwm values from ThrustAllocator
        # self.thrusterControl.setThrusters(thrustValues=thruster_pwm)

    def control_orientation(self):
        roll_output = self.roll_pid.compute(setpoint=self.desired_roll, current_value=self.current_roll, dt = self.ori_freq)
        pitch_output = self.pitch_pid.compute(setpoint=self.desired_pitch, current_value=self.current_pitch, dt = self.ori_freq)
        yaw_output = self.yaw_pid.compute(setpoint=self.desired_yaw, current_value=self.current_yaw, dt = self.ori_freq)

        thruster_pwm = self.thrustAllocator.getRotationPwm([roll_output, pitch_output, yaw_output])

        self.get_logger().info(f'Thruster PWM Output: {thruster_pwm}')



def main(args=None):
    rclpy.init(args=args)
    pid_node = PIDNode()
    rclpy.spin(pid_node)
    pid_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()