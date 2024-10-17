from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    param_file_path = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=[FindPackageShare('param_example'), '/config/config.yaml'],
            description='Path to the parameter file'
        ),
        Node(
            package='param_example',
            executable='python_dynamic_param_node.py',
            name='python_param_node',
            parameters=[param_file_path]
        )
    ])
