import rclpy
import time
from controls_movement.read_depth import DepthReader
from controls_movement.thruster_allocator import ThrustAllocator 

class DepthController:
    def __init__(self, depth_reader, thrust_allocator, kp=1.0, ki=0.0, kd=0.0):
        self.depth_reader = depth_reader
        self.thrust_allocator = thrust_allocator
        
        # PID coefficients
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def maintain_depth(self, target_depth):
        # Ensure ROS2 is initialized
        rclpy.spin_once(self.depth_reader)
        
        # Current depth
        current_depth = self.depth_reader.get_depth()
        if current_depth is None:
            return None  # Wait until depth data is available

        # Calculate error
        error = target_depth - current_depth
        
        # Calculate time difference
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        # PID calculations
        self.integral += error * delta_time
        derivative = (error - self.prev_error) / delta_time if delta_time > 0 else 0.0
        self.prev_error = error

        # Compute control output
        z_thrust = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Use the ThrustAllocator to send the thrust to the motors
        self.get_logger().info(f"Depth: {self.depth}")
        self.thrust_allocator.allocate_thrust(x=0.0, y=0.0, z=z_thrust, roll=0.0, pitch=0.0, yaw=0.0)

def main():
    rclpy.init()
    
    # Initialize DepthReader and ThrustAllocator
    depth_reader = DepthReader()
    thrust_allocator = ThrustAllocator()  # Assuming ThrustAllocator is properly instantiated
    
    # Initialize DepthController with PID coefficients
    controller = DepthController(depth_reader, thrust_allocator, kp=1.0, ki=0.1, kd=0.05)
    
    # Target depth to maintain
    target_depth = 5.0  # Adjust target depth as needed
    
    try:
        while rclpy.ok():
            controller.maintain_depth(target_depth)
            time.sleep(0.1)  # Loop interval, adjust as needed
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()