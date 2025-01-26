"""
To run this test manually, run it from _ws directory, as such
Otherwise, [Errno 2] No such file or directory for thrust_map.csv

izen@Izen-Pavillion:~/HornetXAuv_ws$ python3 ./src/h10_controls/controls_movement/controls_movement/thruster_allocator_test.py

"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from controls_movement.thruster_allocator import ThrustAllocator, thruster_names
import time

def test_thruster_matrix(self, t):
    for i in range(7):
        s = thruster_names[i] + "\t"
        for j in range(6):
            s += " " + str(t.parameters[j][i])
    
        self.get_logger().info(s)

def test_translation(self, t):
    ''' Test translation '''
    vfunc = pwm_to_thrust(t)
    self.get_logger().info(f"+X translation: {vfunc(t.getTranslationPwm([10, 0, 0]))}") # Translation in x-axis
    self.get_logger().info(f"+Y translation: {vfunc(t.getTranslationPwm([0, 10, 0]))}") # Translation in y-axis
    self.get_logger().info(f"+Z translation: {vfunc(t.getTranslationPwm([0, 0, 10]))}") # Translation in z-axis

def test_rotation(self, t):
    ''' Test rotation'''
    vfunc = pwm_to_thrust(t)
    self.get_logger().info(f"+Roll rotation: {vfunc(t.getRotationPwm([10, 0, 0]))}") # Roll right
    self.get_logger().info(f"-Roll rotation: {vfunc(t.getRotationPwm([-10, 0, 0]))}") # Roll left
    self.get_logger().info(f"+Pitch rotation: {vfunc(t.getRotationPwm([0, 10, 0]))}") # Pitch up
    self.get_logger().info(f"-Pitch rotation: {vfunc(t.getRotationPwm([0, -10, 0]))}") # Pitch down
    self.get_logger().info(f"+Yaw rotation: {vfunc(t.getRotationPwm([0, 0, 10]))}") # Yaw left 
    self.get_logger().info(f"-Yaw rotation: {vfunc(t.getRotationPwm([0, 0, -10]))}") # Yaw right

def pwm_to_thrust(t):
    def get_thrust(pwm):
        idx = np.searchsorted(t.thrust_map[:, 1], pwm, 'left')
        return t.thrust_map[idx][0]
    
    return np.vectorize(get_thrust)

class ThrusterAllocatorTest(Node):
    def __init__(self, thruster_allocator):
        super().__init__('thruster_test_node')
        t = thruster_allocator

        self.get_logger().info("--- Thruster matrix ---")
        test_thruster_matrix(self, t)
        self.get_logger().info("\n\n")

        self.get_logger().info("--- Thruster translation test---")
        test_translation(self, t)
        self.get_logger().info("\n\n")
        
        self.get_logger().info("--- Thruster rotation test ---")
        test_rotation(self, t)   
        self.get_logger().info("\n\n")


def main(args=None):
    rclpy.init(args=args)
    thruster_allocator_node = ThrustAllocator()
    tester = ThrusterAllocatorTest(thruster_allocator_node)

    executor = MultiThreadedExecutor()
    executor.add_node(thruster_allocator_node)
    executor.add_node(tester)

    executor.spin()

    thruster_allocator_node.destroy_node()
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
