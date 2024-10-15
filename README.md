# ros2_MinimalParamArg

This package provides explanation and examples on how to use parameters and arguments in ROS2 for both, C++ and python Nodes.

You can easily download and install this example package using the following commands:

Create a folder:
```sh
mkdir ros2_ws
cd ros2_ws
```

Download the repo
```sh
git clone https://github.com/pep248/ros2_MinimalParamArg.git
```

Compile the packages:
```sh
colcon build --packages-select param_example
```

Source the environment:
```sh
source install/setup.bash
```

## Load simple parameters

ROS2 has some inbuilt parameters that are easy to load, them being:
- bool
- int
- float/double
- str/string
- byte array
- bool array
- int array
- float/double array
- str/string array

In these cases, we can simply load a parameter through the ros launch file, using the following commands:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=[FindPackageShare('package'), '/config/config.yaml'],
            description='Path to the parameter file'
        ),
        Node(
            package='package',
            executable='node',
            name='node_name',
            parameters=[params]
        )
    ])
```

And the parameter file will have the following structure:

```yaml
node_name:
  ros__parameters:
    param1: "python_param_node: Hello, world!"
    param2: 42
    param3: true
```

It is very important to point out that the `node_name` has to be the same as the name of the node that is going to load the parameters.

It is also very important to point out that afer the `node_name` there has to be a `ros__parameters` tag, which is necessary to load the parameters.

Here we will provide some examples on how to then use said parameters in the launched nodes:


### Python

We can get the parameters that we have previoulsy loaded using the launchfile using the following python command:

```python
param = self.declare_parameter('param_name', 'default_param_value').get_parameter_value().type_value
```

where you can replace the `value_type`" with any of the following ones:

- bool_value
- integer_value
- double_value
- string_value
- byte_array_value
- bool_array_value
- integer_array_value
- double_array_value
- string_array_value

You can try the python example to load simple parameters using the following command;

```sh
ros2 launch param_example python_param.launch.py
```

### C++

We can get the parameters that we have previoulsy loaded using the launchfile using the following c++ command:

```cpp
auto param = this->declare_parameter<value_type>("param_name", "default_param_value");
```

where you can replace the `value_type`" with any of the following ones:

- `bool`
- `int64_t`
- `double`
- `std::string`
- `std::vector<uint8_t>`
- `std::vector<bool>`
- `std::vector<int64_t>`
- `std::vector<double>`
- `std::vector<std::string>`

You can try the C++ example to load simple parameters using the following command;

```sh
ros2 launch param_example cpp_param.launch.py
```


## Load complex parameters

When we need to use more complex parameters, we will need to load the yaml file directly inside the node. To do this, we will pass the file name as a parameter to the node and then load it. To do so, instead of passing the parameter directly to the node, we will pass the parameter as a dictionary.

To do so, we can use the following launc file structure:

```python
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
            package='package',
            executable='node',
            name='node_name',
            parameters=[{'complex_config_file': param_file_path}]
        )
    ])

```

And we can use any yaml file that we want, without respecting the previous criteria regarding its structure. For example:

```yaml
pose_stamped:
  header:
    frame_id: "world"
  pose:
    position:
      x: 1.0
      y: 2.0
      z: 3.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0

```

were we can use the `pose_stamped` parameter in the node as a dictionary.

We will now provide examples on how to import these complex parameters into a node for the Python and the C++ cases:


### Python

We first need to declare a function that is able to load the yaml, and convert the message from a dictionary into a normal Posestamped message:

```python
    def load_pose_stamped(self, config_file: str) -> PoseStamped:
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
        
        pose_stamped_dict = config['pose_stamped']
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = pose_stamped_dict['header']['frame_id']
        pose_stamped.pose.position.x = pose_stamped_dict['pose']['position']['x']
        pose_stamped.pose.position.y = pose_stamped_dict['pose']['position']['y']
        pose_stamped.pose.position.z = pose_stamped_dict['pose']['position']['z']
        pose_stamped.pose.orientation.x = pose_stamped_dict['pose']['orientation']['x']
        pose_stamped.pose.orientation.y = pose_stamped_dict['pose']['orientation']['y']
        pose_stamped.pose.orientation.z = pose_stamped_dict['pose']['orientation']['z']
        pose_stamped.pose.orientation.w = pose_stamped_dict['pose']['orientation']['w']
```

We secondly need to import the file_path as a parameter and the pass it trhough the previously defined function:

```python
        complex_config_file = self.declare_parameter('complex_config_file', '/random/default/path').get_parameter_value().string_value
        self.pose_stamped = self.load_pose_stamped(complex_config_file)
```

You can try the python example to load simple parameters using the following command;

```sh
ros2 launch param_example python_complex_param.launch.py
```


### C++

We first need to declare a function that is able to load the yaml, and convert the message from a dictionary into a normal Posestamped message:

```python
    def load_pose_stamped(self, config_file: str) -> PoseStamped:
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
        
        pose_stamped_dict = config['pose_stamped']
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = pose_stamped_dict['header']['frame_id']
        pose_stamped.pose.position.x = pose_stamped_dict['pose']['position']['x']
        pose_stamped.pose.position.y = pose_stamped_dict['pose']['position']['y']
        pose_stamped.pose.position.z = pose_stamped_dict['pose']['position']['z']
        pose_stamped.pose.orientation.x = pose_stamped_dict['pose']['orientation']['x']
        pose_stamped.pose.orientation.y = pose_stamped_dict['pose']['orientation']['y']
        pose_stamped.pose.orientation.z = pose_stamped_dict['pose']['orientation']['z']
        pose_stamped.pose.orientation.w = pose_stamped_dict['pose']['orientation']['w']
```

We secondly need to import the file_path as a parameter and the pass it trhough the previously defined function:

```python
        complex_config_file = self.declare_parameter('complex_config_file', '/random/default/path').get_parameter_value().string_value
        self.pose_stamped = self.load_pose_stamped(complex_config_file)
```

You can try the python example to load simple parameters using the following command;

```sh
ros2 launch param_example python_complex_param.launch.py
```

## Pass Arguments



## Author
* Josep Rueda Collell: rueda_999@hotmail.com


