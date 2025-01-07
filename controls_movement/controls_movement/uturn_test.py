"""
This code is used to test the u turn functionality 

It will basically start trying to u turn every 10 seconds or so

This is so that we can run the code and disconnect from the computer

And every 10 seconds, it will select a new desired Yaw to move towards and do PID on that
"""

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

        #Subscribe to Roll, Pitch, Yaw data
        self.subscriptionIMU = self.create_subscription(
            IMU, #custom IMU msg type
            '/sensors/imu',  
            self.imu_callback,
            10
        )

        # Latest sensor readings
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0


        ############################################################################
        ############################################################################
        # PID parameters

        self.desired_yaw = 0.0
        self.yaw_tolerance = 2.0

        self.roll_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        self.pitch_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        self.yaw_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)

        self.PID_dt = 0.5


        ############################################################################
        ############################################################################


        self.thrustAllocator = ThrustAllocator()
        self.thrusterControl = ThrusterControl()
        self.last_time = None


        # We initiate a new test every 10 seconds
        self.isYawTesting = False
        self.yaw_test_timer = self.create_timer(10.0, self.start_yaw_test)

        self.pid_yaw_timer = self.create_timer(self.PID_dt, self.PID_yaw)

    def start_yaw_test(self):
        self.desired_yaw = (self.desired_yaw + 180.0) % 360.0
        self.get_logger().info(f"Starting yaw test to {self.desired_yaw} degrees")
        self.isYawTesting = True

    def PID_yaw(self):
        if not self.isYawTesting:
            return
        
        error = self.desired_yaw - self.current_yaw

        self.get_logger().info(f'Current Yaw: {self.current_yaw}')

        if abs(error) <= self.yaw_tolerance:
            self.get_logger().info("Yaw target reached")
            self.isYawTesting = False
            return
        
        yaw_output = self.yaw_pid.compute(setpoint=self.desired_yaw, current_value=self.current_yaw, dt = self.PID_dt)
        
        thruster_pwm = self.thrustAllocator.getRotationPwm([0.0, 0.0, yaw_output])

        self.get_logger().info(f'Thruster PWM Output: {thruster_pwm}')

        self.thrusterControl.setThrusters(thrustValues=thruster_pwm)


    def imu_callback(self, msg):
        self.current_roll = msg.roll
        self.current_pitch = msg.pitch
        self.current_yaw = msg.yaw

    def control_orientation(self):
        roll_output = self.roll_pid.compute(setpoint=self.desired_roll, current_value=self.current_roll, dt = self.ori_freq)
        pitch_output = self.pitch_pid.compute(setpoint=self.desired_pitch, current_value=self.current_pitch, dt = self.ori_freq)
        yaw_output = self.yaw_pid.compute(setpoint=self.desired_yaw, current_value=self.current_yaw, dt = self.ori_freq)

        thruster_pwm = self.thrustAllocator.getRotationPwm([roll_output, pitch_output, yaw_output])

        self.get_logger().info(f'Thruster PWM Output: {thruster_pwm}')

        self.thrusterControl.setThrusters(thrustValues=thruster_pwm)



def main(args=None):
    rclpy.init(args=args)
    pid_node = PIDNode()
    rclpy.spin(pid_node)
    pid_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()