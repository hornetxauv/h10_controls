from controls_movement.thruster_allocator import ThrustAllocator
# not sure where the thrusters.py file will be located  for the below import vvv
from thrusters.thrusters import ThrusterControl
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
        self.publisher = self.create_publisher(
            Int32MultiArray, 
            'thruster_control_topic', 
            10)
        
        #still need to add compatibility for the roll pitch yaw data
        self.subscription = self.create_subscription(
            Float32,
            '/sensors/depth',  # replace with topic name from os_comms side, this will likely be the depth topic
            self.sensor_callback,
            10
        )
        self.depth_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        self.thrustAllocator = ThrustAllocator()
        self.thrusterControl = ThrusterControl()
        self.last_time = None

    def sensor_callback(self, msg):
        '''
        dt needs to be calculated dynamically, as the duration between each publish to the imu ros topic may vary
        the following code uses the timestamp that comes with each ros msg to calculate dt
        '''
        # Extract the timestamp from the message header
        current_time = self.get_clock().now().to_msg()
        current_seconds = current_time.sec + current_time.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_seconds
            return

        dt = current_seconds - self.last_time
        self.last_time = current_seconds

        # Compute PID output
        pid_output = self.depth_pid.compute(setpoint=0, current_value=msg.data, dt=dt)

        thruster_pwm = self.thrustAllocator.getTranslationPwm([0, 0, pid_output])

        #To Add: publish pid_output to corresponding ros topic
        self.get_logger().info(f'Thruster PWM Output: {thruster_pwm}')
        self.get_logger().info(f'dt: {dt}')

        # set thruster values to the computed pwm values from ThrustAllocator
        self.thrusterControl.setThrusters(thrustValues=thruster_pwm)


def main(args=None):
    rclpy.init(args=args)
    pid_node = PIDNode()
    rclpy.spin(pid_node)
    pid_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()