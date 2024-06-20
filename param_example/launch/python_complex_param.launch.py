from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    param_file_path = LaunchConfiguration('complex_config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'complex_config_file',
            default_value=[FindPackageShare('param_example'), '/config/complex_config.yaml'],
            description='Path to the parameter file'
        ),
        Node(
            package='param_example',
            executable='python_complex_param_node.py',
            name='python_complex_param_node',
            parameters=[{'complex_config_file': param_file_path}]
        )
    ])
