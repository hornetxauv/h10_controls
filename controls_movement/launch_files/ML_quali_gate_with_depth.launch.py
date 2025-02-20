#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# This function is always needed
def generate_launch_description():
    #direction_test_config_path = PathJoinSubstitution(
    #    [FindPackageShare('controls_movement'), 'config', 'simple_direction_test.yaml']
    #)
    quali_gate_pid_config_path = PathJoinSubstitution(
        [FindPackageShare('controls_movement'), 'config', 'quali_gate_pid.yaml']
    )
    vert_pid_config_path = PathJoinSubstitution(
        [FindPackageShare('controls_movement'), 'config', 'vert_pid_test.yaml']
    )
    thruster_config_path = PathJoinSubstitution(
        [FindPackageShare('controls_movement'), 'config', 'thruster.yaml']
    )
    # main_run = Node(package="controls_movement", executable="moveLeft") 
    # change this line to move different direction
    #TODO make it a parameter
    ld = [
        Node(package="can_handler", executable="can_handler"),
        Node(package="controls_movement", executable="movementControls", parameters=[thruster_config_path]),
        Node(package="controls_movement", executable="vertPID", parameters=[vert_pid_config_path]),
        #Node(package="controls_movement", executable="integratedDirTest", parameters=[direction_test_config_path])
        # Node(package="controls_movement", executable="qualiGate", parameters=[quali_gate_pid_config_path]),
        # Node(package="quali_gate_detector", executable="obj_detector")
    ]
    return LaunchDescription(ld)
    
