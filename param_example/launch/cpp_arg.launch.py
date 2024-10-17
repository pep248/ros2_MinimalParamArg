from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'param1',
            default_value="cpp_arg_node: Hello, world!",  # Assuming "default_param1" is a valid default
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
            executable='cpp_param_node',
            name='cpp_arg_node',
            parameters=[{
                'param1': ParameterValue(LaunchConfiguration('param1'), value_type=str),
                'param2': ParameterValue(LaunchConfiguration('param2'), value_type=int),
                'param3': ParameterValue(LaunchConfiguration('param3'), value_type=bool)
            }]
        )
    ])
