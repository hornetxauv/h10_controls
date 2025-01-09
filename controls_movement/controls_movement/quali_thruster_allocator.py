from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   #all of the lines involving ThrusterControl will not work if you have not properly installed virtual CAN
from control_panel.control_panel import create_control_panel #this is a package in PL repo
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

values = {
    'z offset * 10': [10, 100]
}
# value, maximum, smaller_than
create_control_panel(values)

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
    def __init__(self):
        super().__init__('pid_node')
        
        #Subscribe to X, Z error data
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'perc/quali_gate',
            self.error_callback,
            10
        )
        
        # Current errors that will be updated every time ros topic is published to
        self.x_error = 0.0
        self.z_error = 0.0

        ############################################################################
        ############################################################################
        # PID parameters

        self.x_PID = PIDController(Kp = 1.0, Ki = 0.01, Kd = 0.4)
        self.z_PID = PIDController(Kp = 1.0, Ki = 0.01, Kd = 0.4)

        ############################################################################
        ############################################################################
        
        
        self.thrustAllocator = ThrustAllocator()

        self.thrusterControl = ThrusterControl()

        self.last_time = None


    def error_callback(self, msg):
        x_error = msg.data[0]  #need to see how P-L side gonna structure the message
        z_error = msg.data[1]        
        distance = msg.data[2]

        self.get_logger().info(f'x_error: {x_error}, z_error: {z_error}, distance: {distance}')

        x_output = 0.0
        z_output = 0.0
        y_output = 10.0

        # if distance != 0:
        #     # Extract the timestamp from the message header
        #     current_time = self.get_clock().now().to_msg()
        #     current_seconds = current_time.sec + current_time.nanosec * 1e-9

        #     if self.last_time is None:
        #         self.last_time = current_seconds
        #         return #dt is still zero, so do not do PID yet
            
        #     dt = current_seconds - self.last_time
        #     self.last_time = current_seconds

        #     x_output = self.x_PID.compute(setpoint=0.0, current_value=x_error, dt = dt)
        #     z_output = self.z_PID.compute(setpoint=0.0, current_value=z_error, dt = dt)
        #     y_output = 1.0 # always be moving forward, this will need to change once we figure out how to determine if the gate has been passed (?)
        
        z_output += values['z offset * 10'][0]/10

        thruster_pwm = self.thrustAllocator.getTranslationPwm([x_output, y_output, z_output])

        self.get_logger().info(f'x_output: {x_output}, z_output: {z_output}, y_output: {y_output}')
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
