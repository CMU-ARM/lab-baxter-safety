import yaml
import numpy as np
import rospy

from std_msgs.msg import (
    Empty
)

from baxter_core_msgs.msg import (
    EndpointState
)

from sensor_msgs.msg import (
    JointState
)

# Uncomment and use lines below if you want to edit parameters.yml through a dict here
"""
dct =  {"joint_velocity": {"left_s0": 0, "left_s1": 0, "left_e0": 0, "left_e1": 0, "left_w0": 0, "left_w1": 0, "left_w2": 0,
                          "right_s0": 0, "right_s1": 0, "right_e0": 0, "right_e1": 0, "right_w0": 0, "right_w1": 0, "right_w2": 0}, 

        "joint_position": {"left_s0": {"min": 0, "max": 0}, "left_s1": {"min": 0, "max": 0}, "left_e0": {"min": 0, "max": 0}, 
                           "left_e1": {"min": 0, "max": 0}, "left_w0": {"min": 0, "max": 0}, "left_w1": {"min": 0, "max": 0},
                           "left_w2": {"min": 0, "max": 0}, "right_s0": {"min": 0, "max": 0}, "right_s1": {"min": 0, "max": 0}, 
                           "right_e0": {"min": 0, "max": 0}, "right_e1": {"min": 0, "max": 0}, "right_w0": {"min": 0, "max": 0}, 
                           "right_w1": {"min": 0, "max": 0}, "right_w2": {"min": 0, "max": 0}},
 
        "endpoint_velocity": {"left": {"linear": {"x": 0, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}},
                              "right": {"linear": {"x": 0, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}}},

        "endpoint_position": {"left": {"x": {"min": 0, "max": 0}, "y": {"min": 0, "max": 0}, "z": {"min": 0, "max": 0}},
                              "right": {"x": {"min": 0, "max": 0}, "y": {"min": 0, "max": 0}, "z": {"min": 0, "max": 0}}},

        "endpoint_orientation": {"left": {"x": {"min": 0, "max": 0}, "y": {"min": 0, "max": 0}, "z": {"min": 0, "max": 0}, "w": {"min": 0, "max": 0}},
                                 "right": {"x": {"min": 0, "max": 0}, "y": {"min": 0, "max": 0}, "z": {"min": 0, "max": 0}, "w": {"min": 0, "max": 0}}}
}

with open("parameters.yml", "w") as f:
    yaml.dump(dct, f)
"""

# parse parameters.yml file
with open("parameters.yml", 'r') as stream:
    try:
        params = yaml.load(stream)
    except yaml.YAMLError as exc:
        print(exc)

print(None == params["endpoint_orientation"]["left"]["7"]["min"])
