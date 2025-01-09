from .quali_thruster_allocator import PIDController, camera_feed_to_pwm
from controls_movement.thruster_allocator import ThrustAllocator

x_PID = PIDController(Kp = 1.0, Ki = 0.01, Kd = 0.4)
z_PID = PIDController(Kp = 1.0, Ki = 0.01, Kd = 0.4)
thrustAllocator = ThrustAllocator()

