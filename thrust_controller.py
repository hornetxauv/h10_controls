from thruster_allocation import ThrustAllocator

class ThrustController(ThrustAllocator):
    # Relative to camera feed angle
    def calculate_lateral_thrust(self, targetXYZ, targetRPY):
                
