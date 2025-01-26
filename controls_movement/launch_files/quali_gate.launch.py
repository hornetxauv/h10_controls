#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    include_launch_description,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


# This function is always needed
def generate_launch_description():
    thruster_config_path = PathJoinSubstitution(
        [FindPackageShare('controls_movement'), 'config', 'thruster.yaml']
    )
    pid_config_path = PathJoinSubstitution(
        [FindPackageShare('controls_movement'), 'config', 'pid.yaml']
    )
    # main_run = Node(package="controls_movement", executable="moveLeft") 
    # change this line to move different direction
    #TODO make it a parameter
    ld = [
        Node(package="quali_gate_detector", executable="detector"),
        Node(package="quali_gate_detector", executable="detector_listener"),
        Node(package="controls_movement", executable="qualiGate", parameters=[thruster_config_path, pid_config_path]),
        # Node(package="controls_movement", executable="qualiGate_pub"),
    ]
    return LaunchDescription(ld)
    