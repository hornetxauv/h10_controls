import numpy as np
import pandas as pd
from scipy import optimize
from ament_index_python.packages import get_package_share_directory
from controls_movement.param_helper import read_pid_yaml_and_generate_parameters
import rclpy
from rclpy.node import Node

'''
Thruster positions are relative to the CG of the hull

The 7 thrusters are ordered
- Horizontal Thrusters: FL, FR, RL, RR
- Vertical Thrusters: FL, FR, RM

Direction convention is from the POV of the camera: 
- +X is right
- +Y is forward
- +Z is up
'''

thruster_names = np.array(
    ["Hori Front Left", "Hori Front Right", "Hori Rear Left", "Hori Rear Right", "Vert Front Left", "Vert Front Right", "Vert Rear Middle"]
)

class ThrustAllocResult:
    def __init__(self, thrusts, status):
        self.thrusts = thrusts
        self.solveSuccess = status

thruster_positions = np.array(
    [
        [-0.17887, 0.22066, 0.0095],       # Front Left
        [0.176767, 0.222763, 0.0095],      # Front Right
        [-0.176767, -0.222763, 0.0095],    # Rear Left
        [0.176767, -0.222763, 0.0095],     # Rear Right
        [-0.207469, 0.119782, -0.054804],  # Vert Front Left
        [0.207469, 0.119782, -0.054804],   # Vert Front Right
        [0, -0.239564, -0.054804],         # Vert Rear Middle
    ]
)

thruster_directions = np.array(
    [
        [-1, -1, 0],                  # Front Left
        [1, -1, 0],                 # Front Right
        [1, -1, 0],                 # Rear Left
        [-1, -1, 0],                  # Rear Right
        [0, 0, -1],                  # Vert Front Left
        [0, 0, -1],                  # Vert Front Right
        [0, 0, -1],                  # Vert Rear Middle
    ]
)

thruster_directions = thruster_directions / np.linalg.norm(
    thruster_directions, keepdims=True, axis=1
)

class ThrustAllocator(Node):
    '''
    ThrustAllocator solves the matrix Ax = b, where
    A (parameters) is a 6 x 7 matrix of unit xyz force, unit rpy,
    b (output) is the expected xyz force, rpy
    x is the force for each thrusters
    '''
    def __init__(
        self,
        thruster_positions=thruster_positions,
        thruster_directions=thruster_directions
        ):

        super().__init__('thruster_allocator_node')
        package_directory = get_package_share_directory('controls_movement')
        self.declare_parameter('config_location', rclpy.Parameter.Type.STRING)
        config_location = package_directory + self.get_parameter('config_location').get_parameter_value().string_value
        self.declare_parameters(namespace='', parameters=read_pid_yaml_and_generate_parameters('thruster_allocator_node', config_location))
        
        thrust_map_path = package_directory +  self.get_parameter('thrust_map_location').get_parameter_value().string_value #putting this here to try avoid [Errno 2] No such file or directory for thrust_map.csv
        thrust_map = pd.read_csv(thrust_map_path, sep=',', header=None).values
        # thrust_map = pd.read_csv(f"src/controls/controls_movement/config/thrust_map.csv")
        self.thrust_map = thrust_map
        self.thruster_positions = thruster_positions
        self.thruster_directions = thruster_directions
        
        # thruster_biases = np.array([self.get_value('FL')/100,   # Front Left
        #             self.get_value('FR')/100,    # Front Right
        #             self.get_value('RL')/100,    # Rear Left
        #             self.get_value('RR')/100,   # Rear Right
        #             self.get_value('ML')/100,    # Middle Left
        #             self.get_value('MR')/100,    # Middle Right
        #             self.get_value('MM')/100,]) # Middle Middle

        self.parameters = self.initCoefficientMatrix()


    def get_value(self, param_name: str):
        return self.get_parameter(param_name).get_parameter_value().double_value

    # Create coefficient matrix
    def initCoefficientMatrix(self):
        unit_torque = np.cross(self.thruster_positions, self.thruster_directions).T
        unit_rpy = np.array([unit_torque[1], unit_torque[0], unit_torque[2]])
        return np.concatenate(
            (self.thruster_directions.T, unit_rpy)
        )
    

    def getThrusts(self, target_xyz_force, target_rpy):
        output = self.getOutputMatrix(target_xyz_force, target_rpy)
        min_thrust = self.thrust_map[0][0]
        max_thrust = self.thrust_map[-1][0]
        thrust_bound = (min_thrust, max_thrust)

        output = optimize.lsq_linear(self.parameters, output, thrust_bound)
        thrusts = output.x

        # self.get_logger().info(f"Thruster Allocator: {output.success} status:{output.status}")

        if output.status == 3:
            solveSuccess = True
        else:
            solveSuccess = False

        return ThrustAllocResult(thrusts, solveSuccess)
        
    
    def getOutputMatrix(self, target_xyz_force, target_rpy):
        target_xyz_force = np.array(target_xyz_force)
        target_rpy = np.array(target_rpy)
        
        return np.append(target_xyz_force, target_rpy)

    
    def thrustToPwm(self, thrust_forces):
        pwm = []
        thruster_biases = np.array([self.get_value('FL')/100,   # Front Left
                    self.get_value('FR')/100,    # Front Right
                    self.get_value('RL')/100,    # Rear Left
                    self.get_value('RR')/100,   # Rear Right
                    self.get_value('ML')/100,    # Middle Left
                    self.get_value('MR')/100,    # Middle Right
                    self.get_value('MM')/100,]) # Middle Middle
        for force, bias in zip(thrust_forces, thruster_biases):
            force *= bias
            self.get_logger().info(f"bias {bias} force:{force}")
            idx = np.searchsorted(self.thrust_map[:, 0], force, 'left')
            pwm.append(self.thrust_map[idx][1].astype(int))

        # if pwm is between 118 and 137, default it to 127, since that range is all no spin range
        pwm = [127 if 118 <= x <= 137 else x for x in pwm]
        
        
        return pwm
    
    def getThrustPwm(self, target_xyz_force, target_rpy):
        thrust_result = self.getThrusts(target_xyz_force, target_rpy)
        thrust_forces = thrust_result.thrusts

        #if eqn is unsolvable, default to no thruster spin, let buoyancy of floats correct the robot
        if (thrust_result.solveSuccess == False):
            thrust_result.thrusts = [127, 127, 127, 127, 127, 127, 127]
        else:
            thrust_result.thrusts = self.thrustToPwm(thrust_forces)



        return thrust_result
    
    # Pure translation
    def getTranslationPwm(self, target_xyz_force):
        return self.getThrustPwm(target_xyz_force, np.zeros(3))

    # Pure rotation
    def getRotationPwm(self, target_rpy):
        return self.getThrustPwm(np.zeros(3), target_rpy)