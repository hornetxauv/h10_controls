from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pid_config_path = PathJoinSubstitution(
        [FindPackageShare('controls_movement'), 'config', 'pid.yaml']
    )

    fake_pid_node = Node(
        package='controls_movement',
        executable='sim_control_panel',
        name='pid_node',            
        parameters=[pid_config_path]
    )

    return LaunchDescription([
        fake_pid_node
    ])
