from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'param1',
            default_value="python_param_node: Hello, world!",  # Assuming "default_param1" is a valid default
            description='Parameter 1'
        ),
        DeclareLaunchArgument(
            'param2',
            default_value="42",
            description='Parameter 2'
        ),
        DeclareLaunchArgument(
            'param3',
            default_value="true",
            description='Parameter 3'
        ),
        
        Node(
            package='param_example',
            executable='python_param_node.py',
            name='python_param_node',
            parameters=[
                {'param1': LaunchConfiguration('param1')},
                {'param2': LaunchConfiguration('param2')},
                {'param3': LaunchConfiguration('param3')}
            ]
        )
    ])
