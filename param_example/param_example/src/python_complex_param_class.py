#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import os

class MyPythonComplexParamClass(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        complex_config_file = self.declare_parameter('complex_config_file', '/random/default/path').get_parameter_value().string_value
        self.pose_stamped = self.load_pose_stamped(complex_config_file)
        self.get_logger().info(f"PoseStamped: {self.pose_stamped}")

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

        return pose_stamped

    def start(self):
        self.get_logger().info(f"Header: {self.pose_stamped.header}")
        self.get_logger().info(f"Pose:\n  Position: ({self.pose_stamped.pose.position.x}, {self.pose_stamped.pose.position.y}, {self.pose_stamped.pose.position.z})")
        self.get_logger().info(f"  Orientation: ({self.pose_stamped.pose.orientation.x}, {self.pose_stamped.pose.orientation.y}, {self.pose_stamped.pose.orientation.z}, {self.pose_stamped.pose.orientation.w})")
