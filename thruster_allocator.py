import numpy as np
import pandas as pd
from scipy import optimize

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


thrust_map = pd.read_csv("./thrust_map.csv", sep=',', header=None).values

'''
ThrustAllocator solves the matrix Ax = b, where
A (parameters) is a 6 x 7 matrix of unit xyz force, rpy torque,
b (output) is the expected xyz force, rpy torque
x is the force for each thrusters
'''
class ThrustAllocator:
    def __init__(
        self,
        thruster_positions=thruster_positions,
        thruster_directions=thruster_directions,
        thrust_map=thrust_map
    ):
        self.thruster_positions = thruster_positions
        self.thruster_directions = thruster_directions
        self.thrust_map = thrust_map
    
        self.unit_torque = np.cross(self.thruster_positions, self.thruster_directions)
        self.parameters = np.concatenate(
            (self.thruster_directions.T, self.unit_torque.T)
        )

    def getThrusts(self, target_xyz_force, target_torque):
        output = self.getOutputMatrix(target_xyz_force, target_torque)
        min_thrust = self.thrust_map[0][0]
        max_thrust = self.thrust_map[-1][0]
        thrust_bound = (min_thrust, max_thrust)

        return optimize.lsq_linear(self.parameters, output, thrust_bound).x

    
    def getOutputMatrix(self, target_xyz_force, target_torque):
        target_xyz_force = np.array(target_xyz_force)
        target_torque = np.array(target_torque)
         
        return np.append(target_xyz_force, target_torque)

    # Pure translation
    def getTranslationThrusts(self, target_xyz_force):
        return self.getThrusts(target_xyz_force, np.zeros(3))
    
    # Pure rotation
    def getRotationThrusts(self, target_torque):
        return self.getThrusts(np.zeros(3), target_torque)
    