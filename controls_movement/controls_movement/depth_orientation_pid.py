from controls_movement.thruster_allocator import ThrustAllocator
from thrusters.thrusters import ThrusterControl   #all of the lines involving ThrusterControl will not work if you have not properly installed virtual CAN
from .pid_controller import PIDController
from msg_types.msg import DepthIMU, Controls
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
from geometry_msgs.msg import Vector3
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
    'depth Kp': CPI(value=0, maximum=20, minimum=0, multiplier=10),
    'depth Ki': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'depth Kd': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'roll Kp': CPI(value=0.1, maximum=1, minimum=0, multiplier=100),
    'roll Ki': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'roll Kd': CPI(value=0.1, maximum=1, minimum=0, multiplier=100),
    'pitch Kp': CPI(value=0.1, maximum=1, minimum=0, multiplier=100),
    'pitch Ki': CPI(value=0, maximum=1, minimum=0, multiplier=100),
    'pitch Kd': CPI(value=0.1, maximum=1, minimum=0, multiplier=100)
}
# can't seem to use simultaneously with thruster biases control panel... sometimes. idk.
create_control_panel("Depth Orientation PID", values)

class DepthOriPIDNode(Node):
    def __init__(self):
        super().__init__('depth_ori_pid_node')

        #Subscribe to Roll, Pitch, Yaw data
        self.subscription = self.create_subscription(
            DepthIMU, #custom IMU msg type
            '/sensors/depth_imu',  
            self.depth_imu_callback,
            10
        )

        self.publisher = self.create_publisher(Controls, "/controls/wanted_depth_ori", 10)

        # Latest sensor readings
        self.current_depth = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0

        ############################################################################
        ############################################################################
        # PID parameters

        self.desired_depth = -values['desired_depth'].value # Example depth to maintain
        self.desired_roll = 0.0    # Example orientation targets
        self.desired_pitch = 0.0

        # self.depth_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        # self.roll_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        # self.pitch_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        
        self.depth_pid = PIDController(Kp=values['depth Kp'].value, Ki=values['depth Ki'].value, Kd=values['depth Kd'].value)
        self.roll_pid = PIDController(Kp=values['roll Kp'].value, Ki=values['roll Ki'].value, Kd=values['roll Kd'].value)
        self.pitch_pid = PIDController(Kp=values['pitch Kp'].value, Ki=values['pitch Ki'].value, Kd=values['pitch Kd'].value)

        self.controls_message = Controls()

        ############################################################################
        ############################################################################

        self.last_time = None
        self.ori_last_time = None

    def depth_imu_callback(self, msg):
        self.current_depth = msg.depth
        self.current_roll = msg.roll
        self.current_pitch = msg.pitch

        current_time = self.get_clock().now().to_msg()
        current_seconds = current_time.sec + current_time.nanosec * 1e-9 #? sending only 60Hz why nanosec change to milli
        if self.ori_last_time is None:
            self.ori_last_time = current_seconds
            return #dt is still zero, so do not do PID yet

        # self.get_logger().info("Here")
        
        dt = current_seconds - self.ori_last_time
        self.ori_last_time = current_seconds
        self.depth_ori_control(dt)

    def assign_vector(self, vector, values):
        vector.x = values[0]
        vector.y = values[1]
        vector.z = values[2]

    def depth_ori_control(self, dt):
        #these lines are needed because of control panel, once control panel no need, these can be removed
        self.depth_pid.update_consts(new_Kp=values['depth Kp'].value, new_Ki=values['depth Ki'].value, new_Kd=values['depth Kd'].value)
        self.desired_depth = -values['desired_depth'].value
        self.roll_pid.update_consts(new_Kp=values['roll Kp'].value, new_Ki=values['roll Ki'].value, new_Kd=values['roll Kd'].value)
        # self.roll_pid.update_consts(new_Kp=values['roll Kp'].value, new_Ki=values['roll Ki'].value, new_Kd=values['roll Kd'].value)
        # self.get_logger().info(self.pitch_pid.update_consts(new_Kp=values['pitch Kp'].value, new_Ki=values['pitch Ki'].value, new_Kd=values['pitch Kd'].value))
        self.pitch_pid.update_consts(new_Kp=values['pitch Kp'].value, new_Ki=values['pitch Ki'].value, new_Kd=values['pitch Kd'].value)
        self.desired_depth = -values['desired_depth'].value

        depth_output = self.depth_pid.compute(setpoint=self.desired_depth, current_value=self.current_depth, dt=dt, multiplier=values["multiplier"].value)[0]
        roll_output = self.roll_pid.compute(setpoint=self.desired_roll, current_value=self.current_roll, dt = dt, multiplier=values["multiplier"].value)[0]
        pitch_output, error, kd, deri = self.pitch_pid.compute(setpoint=self.desired_pitch, current_value=self.current_pitch, dt = dt, multiplier=values["multiplier"].value)
        
        self.assign_vector(self.controls_message.translation, [0.0, 0.0, depth_output])
        self.assign_vector(self.controls_message.rotation, [roll_output, pitch_output, 0.0])

        # self.get_logger().info(f"LOG Translation: ${self.controls_message.translation} and Rotation ${self.controls_message.rotation}")

        self.publish()

    def publish(self):
        if self.controls_message is not None:
            self.publisher.publish(self.controls_message)
            self.get_logger().info(f"Published Translation: ${self.controls_message.translation} and Rotation ${self.controls_message.rotation}")

def main(args=None):
    rclpy.init(args=args)
    depth_ori_pid_node = DepthOriPIDNode()
    try:
        while True:
            rclpy.spin(depth_ori_pid_node)
    finally:
        depth_ori_pid_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()