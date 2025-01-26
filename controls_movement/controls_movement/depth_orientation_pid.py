from controls_movement.thruster_allocator import ThrustAllocator
# from thrusters.thrusters import ThrusterControl   #all of the lines involving ThrusterControl will not work if you have not properly installed virtual CAN
from .pid_controller import PIDController
from msg_types.msg import DepthIMU, Controls
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
from geometry_msgs.msg import Vector3
from ament_index_python.packages import get_package_share_directory
from controls_movement.param_helper import read_pid_yaml_and_generate_parameters

class DepthOriPIDNode(Node):
    def __init__(self):
        super().__init__('depth_ori_pid_node')
        package_directory = get_package_share_directory('controls_movement')
        self.declare_parameter('config_location', rclpy.Parameter.Type.STRING)
        config_location = package_directory + self.get_parameter('config_location').get_parameter_value().string_value
        self.declare_parameters(namespace='', parameters=read_pid_yaml_and_generate_parameters('depth_ori_pid_node', config_location))

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

        self.desired_depth = -self.get_value('desired_depth') # Example depth to maintain
        self.desired_roll = 0.0    # Example orientation targets
        self.desired_pitch = 0.0

        # self.depth_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        # self.roll_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        # self.pitch_pid = PIDController(Kp=1.0, Ki=0.1, Kd=0.01)
        
        self.depth_pid = PIDController(Kp=self.get_value('depth_Kp'), Ki=self.get_value('depth_Ki'), Kd=self.get_value('depth_Kd'))
        self.roll_pid = PIDController(Kp=self.get_value('roll_Kp'), Ki=self.get_value('roll_Ki'), Kd=self.get_value('roll_Kd'))
        self.pitch_pid = PIDController(Kp=self.get_value('pitch_Kp'), Ki=self.get_value('pitch_Ki'), Kd=self.get_value('pitch_Kd'))

        self.controls_message = Controls()

        ############################################################################
        ############################################################################

        self.last_time = None
        self.ori_last_time = None

    def get_value(self, param_name: str):
        return self.get_parameter(param_name).get_parameter_value().double_value

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
        self.depth_pid.update_consts(new_Kp=self.get_value('depth_Kp'), new_Ki=self.get_value('depth_Ki'), new_Kd=self.get_value('depth_Kd'))
        self.desired_depth = -self.get_value('desired_depth')
        self.roll_pid.update_consts(new_Kp=self.get_value('roll_Kp'), new_Ki=self.get_value('roll_Ki'), new_Kd=self.get_value('roll_Kd'))
        # self.roll_pid.update_consts(new_Kp=self.get_value('roll_Kp'), new_Ki=self.get_value('roll_Ki'), new_Kd=self.get_value('roll_Kd'))
        # self.get_logger().info(self.pitch_pid.update_consts(new_Kp=self.get_value('pitch_Kp'), new_Ki=self.get_value('pitch_Ki'), new_Kd=self.get_value('pitch_Kd')))
        self.pitch_pid.update_consts(new_Kp=self.get_value('pitch_Kp'), new_Ki=self.get_value('pitch_Ki'), new_Kd=self.get_value('pitch_Kd'))
        self.desired_depth = -self.get_value('desired_depth')

        depth_output = self.depth_pid.compute(setpoint=self.desired_depth, current_value=self.current_depth, dt=dt, multiplier=self.get_value("multiplier"))[0]
        roll_output = self.roll_pid.compute(setpoint=self.desired_roll, current_value=self.current_roll, dt = dt, multiplier=self.get_value("multiplier"))[0]
        pitch_output, error, kd, deri = self.pitch_pid.compute(setpoint=self.desired_pitch, current_value=self.current_pitch, dt = dt, multiplier=self.get_value("multiplier"))
        
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