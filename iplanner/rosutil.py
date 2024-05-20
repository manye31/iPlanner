# ==============================================================================
# Copyright <2019> <Chen Wang [https://chenwang.site], Carnegie Mellon University>
# Refer to: https://github.com/wang-chen/interestingness_ros/blob/master/script/rosutil.py
# ==============================================================================

import os
import rclpy
# from rclpy.utilities import load_yaml
import torch
import numpy as np

class ROSArgparse():
    def __init__(self, node : rclpy.node):
        self.node = node

    def add_argument(self, name, default, type=None, help=None):
        name = os.path.join(self.node.get_name(), name)
        if self.node.has_parameter(name):
            self.node.get_logger().info(f"Got param {name}")
        else:
            self.node.get_logger().warn(f"Couldn\'t find param: {name}, Using default: {default}")
        self.node.declare_parameter(name, default)
        value = self.node.get_parameter(name).value
        variable = name[name.rfind('/')+1:].replace('-','_')
        if isinstance(value, str):
            exec('self.%s=\'%s\''%(variable, value))
        else:
            exec('self.%s=%s'%(variable, value))

    def parse_args(self):
        return self
    
    # def load_params_from_yaml(self, file):
    #     with open(file, "r") as f:
    #         yaml_data = f.read()
        
    #     params = load_yaml(yaml_data)

    #     self.set_parameters(params)


def msg_to_torch(data, shape=np.array([-1])):
    return torch.from_numpy(data).view(shape.tolist())


def torch_to_msg(tensor):
    return [tensor.view(-1).cpu().numpy(), tensor.shape]
