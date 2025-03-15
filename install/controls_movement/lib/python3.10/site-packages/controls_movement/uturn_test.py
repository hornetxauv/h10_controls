import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from msg_types.msg import Movement
from custom_msgs.msg import GateDetection

from controls_movement.pid_controller import PIDController

from ament_index_python.packages import get_package_share_directory
from controls_movement.param_helper import read_pid_yaml_and_generate_parameters


class UTurnPIDNode(Node):
    def __init__(self):
        super().__init__('u_turn_node')

        #Subscribe to yaw data 
        self.subscription = self.create_subscription(
            DepthIMU,
            '/sensors/depth_imu',
            self.drpy_callback,
            10
        )

        self.publisher = self.create_publisher(Movement, "/controls/wanted_goal_movement", 10)

        
        # Current errors that will be updated every time ros topic is published to
        self.yaw = None
        self.goal_yaw = None
        # self.z_error = 0.0

        self.yaw_PID = PIDController(Kp=0.01, Ki=0, Kd=0)
        # self.z_PID = PIDController(Kp=self.get_value('z_Kp'), Ki=self.get_value('z_Ki'), Kd=self.get_value('z_Kd'))


        

        self.last_time = None

    def drpy_callback(self, msg):
        self.current_yaw = msg.yaw
        if self.goal_yaw == None:
            self.goal_yaw = (self.current_yaw + 180) % 180

        


    def detection_callback(self, msg):
        self.x_PID.update_consts(new_Kp=self.get_value('x_Kp'), new_Ki=self.get_value('x_Ki'), new_Kd=self.get_value('x_Kd'))

        # Taking to the right to be positive dx
        x_error = msg.dx 
        # z_error = msg.dy # Note: removed due to using depth sensor
        width = msg.width
        self.get_logger().info(f'x_error: {x_error}, distance: {width}')

        # Extract the timestamp from the message header
        current_time = self.get_clock().now().to_msg()
        current_seconds = current_time.sec + current_time.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_seconds
            return #dt is still zero, so do not do PID yet

        dt = current_seconds - self.last_time
        self.last_time = current_seconds

        x_output = 0.0
        # z_output = 0.0
        y_output = 0.0

        # only do PID if there is a gate detected, i.e. distance between gates =/= 0
        if width != 0:
            x_output, xP_term, xI_term, xD_term = self.x_PID.compute(setpoint=0.0, current_value=x_error, dt = dt)
            # z_output = self.z_PID.compute(setpoint=0.0, current_value=z_error, dt = dt)
            # y_output = 1.0 # always be moving forward, this will need to change once we figure out how to determine if the gate has been passed (?)

        self.movement_message = Movement()
        self.movement_message.x = float(x_output)
        self.publish()

    def publish(self):
        if self.movement_message is not None:
            self.publisher.publish(self.movement_message)
            self.get_logger().info(f"Published Translation: ${self.movement_message.x}")


def main(args=None):
    rclpy.init(args=args)
    pid_node = QualiGatePIDNode()

    executor = MultiThreadedExecutor()
    executor.add_node(pid_node)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
