#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class MyPythonDynamicParamClass(Node):
    def __init__(self, node_name:str):
        super().__init__(node_name)
        self.param1 = self.declare_parameter('param1', 'default_param1').get_parameter_value().string_value
        self.param2 = self.declare_parameter('param2', 0).get_parameter_value().integer_value
        self.param3 = self.declare_parameter('param3', False).get_parameter_value().bool_value
        self.add_on_set_parameters_callback(self.param_update_callback)
    
    
    # ros2 param set /python_param_node param1 "\"python_param_node: This is a modified param\""
    # ros2 param set /python_param_node param2 34
    # ros2 param set /python_param_node param3 false
    def param_update_callback(self, parameters):
        result = SetParametersResult(successful=True)

        for param in parameters:
            if param.name == 'param1':
                if param.type_ != param.Type.STRING:
                    result.successful = False
                    result.reason = "param1 must be a string"
                    break
                else:
                    self.param1 = param.value
            elif param.name == 'param2':
                if param.type_ != param.Type.INTEGER:
                    result.successful = False
                    result.reason = "param2 must be an integer"
                    break
                else:
                    self.param2 = param.value
            elif param.name == 'param3':
                if param.type_ != param.Type.BOOL:
                    result.successful = False
                    result.reason = "param3 must be a boolean"
                    break
                else:
                    self.param3 = param.value
            else:
                result.successful = False
                result.reason = f"{param.name} is not a parameter of this node"
                break

        self.print_param()
        return result
    
    
    def start(self):
        self.print_param()
        while(1):
            pass
        
        
    def print_param(self):
        self.get_logger().info(f"param1: {self.param1}")
        self.get_logger().info(f"param2: {self.param2}")
        self.get_logger().info(f"param3: {self.param3}")
