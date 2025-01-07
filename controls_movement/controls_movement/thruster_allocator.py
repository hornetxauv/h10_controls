import numpy as np
import pandas as pd
from scipy import optimize

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
        [1, 1, 0],                  # Front Left
        [-1, 1, 0],                 # Front Right
        [-1, 1, 0],                 # Rear Left
        [1, 1, 0],                  # Rear Right
        [0, 0, 1],                  # Vert Front Left
        [0, 0, 1],                  # Vert Front Right
        [0, 0, 1],                  # Vert Rear Middle
    ]
)

"""
Old Values from H9
Kept as reference jic the mapping was done wrongly
All the code currently caa 2/1/25 is based on the H9 mapping

thruster_positions = np.array(
    [
        [-0.22, 0.238, -0.054],     # Front Left
        [0.22, 0.238, -0.054],      # Front Right
        [-0.22, -0.217, -0.054],    # Rear Left
        [0.22, -0.217, -0.054],     # Rear Right
        [-0.234, 0.22, -0.107],      # Middle Left
        [0.234, 0.22, -0.107],       # Middle Right
        [0.0, -0.22, -0.107],       # Middle Middle
    ]
)

thruster_directions = np.array(
    [
        [1, 1, 0],                  # Front Left
        [-1, 1, 0],                 # Front Right
        [-1, 1, 0],                 # Rear Left
        [1, 1, 0],                  # Rear Right
        [0, 0, 1],                  # Middle Left
        [0, 0, 1],                  # Middle Right
        [0, 0, 1],                  # Middle Middle
    ]
)
"""

thruster_directions = thruster_directions / np.linalg.norm(
    thruster_directions, keepdims=True, axis=1
)

thruster_biases = np.array([1.0,    # Front Left
                            1.0,    # Front Right
                            1.0,    # Rear Left
                            1.0,    # Rear Right
                            1.0,    # Middle Left
                            1.0,    # Middle Right
                            1.4,])  # Middle Middle


thrust_map = pd.read_csv("./src/h10_controls/controls_movement/controls_movement/thrust_map.csv", sep=',', header=None).values
#thrust_map = pd.read_csv("./thrust_map.csv", sep=',', header=None).values
#./src/h10_controls/controls_movement/controls_movement/thrust_map.csv

class ThrustAllocator:
    '''
    ThrustAllocator solves the matrix Ax = b, where
    A (parameters) is a 6 x 7 matrix of unit xyz force, unit rpy,
    b (output) is the expected xyz force, rpy
    x is the force for each thrusters
    '''
    def __init__(
        self,
        thruster_positions=thruster_positions,
        thruster_directions=thruster_directions,
        thrust_map=thrust_map
    ):
        self.thruster_positions = thruster_positions
        self.thruster_directions = thruster_directions
        self.thrust_map = thrust_map
        self.parameters = self.initCoefficientMatrix()

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

        return optimize.lsq_linear(self.parameters, output, thrust_bound).x
    
    def getOutputMatrix(self, target_xyz_force, target_rpy):
        target_xyz_force = np.array(target_xyz_force)
        target_rpy = np.array(target_rpy)
         
        return np.append(target_xyz_force, target_rpy)

    
    def thrustToPwm(self, thrust_forces):
        pwm = []
        for force in thrust_forces:
            idx = np.searchsorted(self.thrust_map[:, 0], force, 'left')
            pwm.append(self.thrust_map[idx][1].astype(int))
        
        return pwm
    
    def getThrustPwm(self, target_xyz_force, target_rpy):
        thrust_forces = self.getThrusts(target_xyz_force, target_rpy)
        pwm = self.thrustToPwm(thrust_forces)

        return pwm
    
    # Pure translation
    def getTranslationPwm(self, target_xyz_force):
        return self.getThrustPwm(target_xyz_force, np.zeros(3))

    # Pure rotation
    def getRotationPwm(self, target_rpy):
        return self.getThrustPwm(np.zeros(3), target_rpy)