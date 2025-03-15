from threading import Thread
from controls_movement.pid_controller import PIDController
from h10_pooltest_ws.src.h10_controls.controls_movement.controls_movement.yaw_pid_hardcode_node import YawPIDNode
from msg_types.msg import DepthIMU
from msg_types.msg import Controls
from msg_types.msg import Movement
from msg_types.msg import PIDoutputs
from msg_types.msg import YawInfo
import time
from msg_types.srv._movement_service import MovementService
import rclpy
from rclpy.executors import MultiThreadedExecutor

from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
from ament_index_python.packages import get_package_share_directory
from controls_movement.param_helper import read_pid_yaml_and_generate_parameters

class YawNode(Node):
    def __init__(self):
        super().__init__('yaw_node')
        #Subscribe to Depth, RPY Data
        self.subscription = self.create_subscription(
            DepthIMU,
            '/sensors/depth_imu',
            self.drpy_callback,
            10
        )
  


        # change back once control panel not needed
        self.depth_pid = PIDController(Kp=self.get_value('depth_Kp'), Ki=self.get_value('depth_Ki'), Kd=self.get_value('depth_Kd'))
        self.roll_pid = PIDController(Kp=self.get_value('roll_Kp'), Ki=self.get_value('roll_Ki'), Kd=self.get_value('roll_Kd'))
        self.pitch_pid = PIDController(Kp=self.get_value('pitch_Kp'), Ki=self.get_value('pitch_Ki'), Kd=self.get_value('pitch_Kd'))
        self.yaw_pid = PIDController(Kp=self.get_value('yaw_Kp'), Ki=self.get_value('yaw_Ki'), Kd=self.get_value('yaw_Kd'), isOri=True)

        ############################################################################
        ############################################################################

        # self.frequency = self.get_value('PID_freq')
        # self.timer_period = 1.0 / self.frequency
        # self.timer = self.create_timer(self.timer_period, self.stationkeep_callback)




    def stationkeep_callback(self):
        self.change_timer_period(self.get_value('PID_freq'))

        current_time = self.get_clock().now().to_msg()
        current_seconds = current_time.sec + current_time.nanosec * 1e-9 #? sending only 60Hz why nanosec change to milli

        if self.last_time is None:
            self.last_time = current_seconds
            return #dt is still zero, so do not do PID yet
        
        dt = current_seconds - self.last_time
        self.last_time = current_seconds

        # self.stationkeep(dt)

    def drpy_callback(self, msg):
        if self.desired_yaw == None:
            self.desired_yaw = msg.yaw
        self.current_depth = msg.depth
        self.current_roll = msg.roll
        self.current_pitch = msg.pitch
        self.current_yaw = msg.yaw

        current_time = self.get_clock().now().to_msg()
        current_seconds = current_time.sec + current_time.nanosec * 1e-9 #? sending only 60Hz why nanosec change to milli

        if self.last_time is None:
            self.last_time = current_seconds
            return #dt is still zero, so do not do PID yet
        
        self.dt = current_seconds - self.last_time
        self.last_time = current_seconds
    
    def get_info(self):
        return [self.current_depth, self.current_roll, self.current_pitch, self.current_yaw, self.dt]

