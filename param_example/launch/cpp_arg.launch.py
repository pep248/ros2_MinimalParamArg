import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction

configurable_parameters = [{'name': 'param1',      'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'param2',      'default': 'camera', 'description': 'namespace for camera'},
                           {'name': 'param3',      'default': "''",     'description': 'choose device by serial number'}]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, params, param_name_suffix=''):
    # param_file_path = LaunchConfiguration('param_file')
    # DeclareLaunchArgument(
    #     'param_file',
    #     default_value=[FindPackageShare('param_example'), '/config/config.yaml'],
    #     description='Path to the parameter file'
    # )
    # _config_file = LaunchConfiguration('config_file' + param_name_suffix).perform(context)
    # param_file_path = LaunchConfiguration('param_file')
    # if param_file_path == "''":
    #     params_from_file = {}
    # else:
    #     params_from_file = yaml_to_dict(param_file_path)

    return [
        Node(
            package='param_example',
            executable='cpp_param_node',
            name='cpp_param_node',
            parameters=[params],
            )
    ]


def generate_launch_description():
    configurable_params = declare_configurable_parameters(configurable_parameters)
    launch_setup_args = {'params': set_configurable_parameters(configurable_parameters)}
    
    return LaunchDescription([
        configurable_params,
        OpaqueFunction(function=launch_setup, kwargs=launch_setup_args)
    ])


# ros2 launch param_example cpp_arg.launch.py param1:="Modified Hello, world"
# ros2 run param_example cpp_param_node --ros-args -p  param1:="Modified Hello, world"