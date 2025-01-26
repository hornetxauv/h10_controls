from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    thruster_config_path = PathJoinSubstitution(
        [FindPackageShare('controls_movement'), 'config', 'thruster.yaml']
    )

    thrust_node = Node(
        package='controls_movement',
        executable='thrust',
        name='thruster_allocator_node',            
        parameters=[thruster_config_path]
    )

    return LaunchDescription([
        thrust_node
    ])
