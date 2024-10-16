#!/usr/bin/env python3
import numpy as np
import rclpy
from controls_core.params import UPTHRUST
from controls_core.thruster_allocator import ThrustAllocator
from controls_core.utilities import BACK, DOWN, FRONT, LEFT, RIGHT, UP
from rclpy.node import Node
from thrusters.thrusters import ThrusterControl

LIN_ACC_MAG = 1.0

thrusterControl = ThrusterControl()
thrustAllocator = ThrustAllocator()


class MovementTestPublisher(Node):
    def __init__(self, lin_acc, angular_acc):
        super().__init__("movement_test_publisher_node")
        self.timer = self.create_timer(1.0, self.movementTest)
        self.lin_acc = lin_acc
        self.angular_acc = angular_acc

    def movementTest(self):
        # FL-FR-ML-MR-RL-RR
        thrustValues = thrustAllocator.getThrustPWMs(self.lin_acc, self.angular_acc)
        self.get_logger().info(f"{thrustValues}")
        thrusterControl.setThrusters(thrustValues=thrustValues)


def main(args, direction, magnitude):
    rclpy.init(args=args)
    node = MovementTestPublisher(
        lin_acc=magnitude * np.array(direction) + np.array([0, 0, UPTHRUST]),
        angular_acc=[0, 0, 0],
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        thrusterControl.killThrusters()
    finally:
        rclpy.try_shutdown()


def moveLeft(args=None):
    main(args, LEFT, LIN_ACC_MAG)


def moveRight(args=None):
    main(args, RIGHT, LIN_ACC_MAG)


def moveFront(args=None):
    main(args, FRONT, LIN_ACC_MAG)


def moveBack(args=None):
    main(args, BACK, LIN_ACC_MAG)


def moveUp(args=None):
    main(args, UP, LIN_ACC_MAG)


def moveDown(args=None):
    main(args, DOWN, LIN_ACC_MAG)