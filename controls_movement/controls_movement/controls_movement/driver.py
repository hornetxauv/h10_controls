#!/usr/bin/env python3
import numpy as np
from controls_core.attitude_control import AttitudeControl
from controls_core.params import rollPID, yawPID
from controls_core.thruster_allocator import ThrustAllocator
from imu_msg.msg import Imu
from rclpy.node import Node
from thrusters.thrusters import ThrusterControl

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()


class Driver(Node):
    def __init__(self) -> None:
        super().__init__("driver_node")
        self.state_subscriber = self.create_subscription(
            Imu, "/sensors/imu", self.attitudeControl, 10
        )
        self.attitudeControl = AttitudeControl(rollPID, yawPID)

        self.linear_acc = np.array([0, 0, 0])
        self.angular_acc = np.array([0, 0, 0])

    def _drive(self, msg: Imu):
        currAttRPY = [msg.roll_pitch_yaw.x, msg.roll_pitch_yaw.y, msg.roll_pitch_yaw.z]
        attCorr = self.attitudeControl.getAttitudeCorrection(
            currAttRPY=currAttRPY, targetAttRPY=[0, 0, 0]
        )

        thrustValues = thrustAllocator.getThrustPWMs(
            self.linear_acc, self.angular_acc + np.array(attCorr)
        )
        thrusterControl.setThrusters(thrustValues=thrustValues)

    def drive(self, linear_acc, angular_acc):
        self.linear_acc = linear_acc
        self.angular_acc = angular_acc