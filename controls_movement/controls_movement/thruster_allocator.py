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


thrust_map = pd.read_csv("./src/h10_controls/controls_movement/controls_movement/thrust_map.csv", sep=',', header=None).values
#./src/h10_controls/controls_movement/controls_movement/thrust_map.csv

class ThrustAllocator:
    '''
    ThrustAllocator solves the matrix Ax = b, where
    A (parameters) is a 6 x 7 matrix of unit xyz force, unit torque vector (in rpy),
    b (output) is the expected xyz force, torque vector
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
        unit_torque = np.cross(self.thruster_positions, self.thruster_directions)
        return np.concatenate(
            (self.thruster_directions.T, unit_torque.T)
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

    
    def thrustToPwm(self, thrust_forces):
        pwm = []
        for force in thrust_forces:
            idx = np.searchsorted(self.thrust_map[:, 0], force, 'left')
            pwm.append(self.thrust_map[idx][1].astype(int))
        
        return pwm
    
    def getThrustPwm(self, target_xyz_force, target_torque):
        thrust_forces = self.getThrusts(target_xyz_force, target_torque)
        pwm = self.thrustToPwm(thrust_forces)

        return pwm
    
    # Pure translation
    def getTranslationPwm(self, target_xyz_force):
        return self.getThrustPwm(target_xyz_force, np.zeros(3))

    # Pure rotation
    def getRotationPwm(self, target_torque):
        return self.getThrustPwm(np.zeros(3), target_torque)