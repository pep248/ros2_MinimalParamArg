#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyPythonParamClass(Node):
    def __init__(self, node_name:str):
        super().__init__(node_name)
        self.param1 = self.declare_parameter('param1', 'default_param1').get_parameter_value().string_value
        self.param2 = self.declare_parameter('param2', 0).get_parameter_value().integer_value
        self.param3 = self.declare_parameter('param3', False).get_parameter_value().bool_value
    
    def start(self):
        self.get_logger().info(f"param1: {self.param1}")
        self.get_logger().info(f"param2: {self.param2}")
        self.get_logger().info(f"param3: {self.param3}")
