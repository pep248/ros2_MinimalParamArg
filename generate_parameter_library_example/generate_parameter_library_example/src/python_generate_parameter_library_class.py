#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Custom parameters
from generate_parameter_library_example.python_config_1_parameters import config_1_params
from generate_parameter_library_example.python_config_2_parameters import config_2_params

class MyPythonGenerateParamLibraryClass(Node):
    def __init__(self, node_name:str):
        super().__init__(node_name)
        
        config1_params_listener = config_1_params.ParamListener(self)
        config1_params = config1_params_listener.get_params()
        
        config2_params_listener = config_2_params.ParamListener(self)
        config2_params = config2_params_listener.get_params()
        
    def start(self):
        self.get_logger().info(f"param1: {self.param1}")
        self.get_logger().info(f"param2: {self.param2}")
        self.get_logger().info(f"param3: {self.param3}")
