from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    direction_test_config_path = PathJoinSubstitution(
        [FindPackageShare('controls_movement'), 'config', 'simple_direction_test.yaml']
    )
    thruster_config_path = PathJoinSubstitution(
        [FindPackageShare('controls_movement'), 'config', 'thruster.yaml']
    )

    simple_direction_test_node = Node(package="controls_movement", executable="dirTest", parameters=[thruster_config_path, direction_test_config_path])
    return LaunchDescription([simple_direction_test_node])
