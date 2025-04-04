#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Custom parameters
from generate_parameter_library_example.python_config_1_parameters import config_1_params
from generate_parameter_library_example.python_config_2_parameters import config_2_params

class MyPythonGenerateParamLibraryClass(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        
        self.config1_params_listener = config_1_params.ParamListener(self)
        self.config1_params = self.config1_params_listener.get_params()
        
        self.config2_params_listener = config_2_params.ParamListener(self)
        self.config2_params = self.config2_params_listener.get_params()
        
    def start(self):
        # Print all parameters from config_1_params
        self.get_logger().info("Config 1 Parameters:")
        self.get_logger().info(f"  param1: {self.config1_params.param1}")
        self.get_logger().info(f"  param2: {self.config1_params.param2}")
        
        # Print all parameters from config_2_params
        self.get_logger().info("Config 2 Parameters:")
        self.get_logger().info(f"  param_field_1.param1: {self.config2_params.param_field_1.param1}")
        self.get_logger().info(f"  param_field_1.param2: {self.config2_params.param_field_1.param2}")
        self.get_logger().info(f"  param_field_2.paramA: {self.config2_params.param_field_2.paramA}")
        self.get_logger().info(f"  param_field_2.paramB: {self.config2_params.param_field_2.paramB}")
