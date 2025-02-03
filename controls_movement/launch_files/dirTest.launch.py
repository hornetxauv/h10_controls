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
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# This function is always needed
def generate_launch_description():
    dirTest_config = PathJoinSubstitution(
        [FindPackageShare('controls_movement'), 'config', 'simple_direction_test.yaml']
    )
    # main_run = Node(package="controls_movement", executable="moveLeft") 
    # change this line to move different direction
    #TODO make it a parameter
    ld = [
        # Node(package="can_handler", executable="can_handler"),
        Node(package="controls_movement", executable="dirTest", parameters=[dirTest_config]),
        # Node(package="controls_movement", executable="vertPID_pub"),
    ]
    return LaunchDescription(ld)
    