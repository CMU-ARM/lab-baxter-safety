#!/usr/bin/python

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
import numpy as np

class SafetyNode(object):
    """
    Monitor the robot and make sure it follows certain constraints,
    if these constratins are violated, the robot shutdowns
    """

    def __init__(self):
        self._estop_pub = rospy.Publisher('/robot/set_super_stop',Empty,queue_size=2)
        self._spin_rate = 10 
        self._spin_rate_control = rospy.Rate(self._spin_rate)
        self._kill_flag = False

        self._left_endpoint_sub = rospy.Subscriber('/robot/limb/left/endpoint_state',EndpointState,self._left_endpoint_cb,queue_size=1)
        self._right_endpoint_sub = rospy.Subscriber('/robot/limb/right/endpoint_state',EndpointState,self._right_endpoint_cb,queue_size=1)
        self._jointstate_sub = rospy.Subscriber('/robot/joint_states',JointState,self._jointstate_cb,queue_size=1)

        self._last_left_endpoint = None
        self._last_right_endpoint = None
        self._last_jointstate = None

    def _left_endpoint_cb(self, msg):
        self._last_left_endpoint = msg

    def _right_endpoint_cb(self, msg):
        self._last_right_endpoint = msg 

    def _jointstate_cb(self, msg):
        self._last_jointstate = msg


    def _hand_endpoint_constraints(self):
        #check constraits

        if self._kill_flag:
            return

        #TODO rewrite this as a config file or something
        if self._last_left_endpoint != None:
            if self._last_left_endpoint.pose.position.z > 1.2:
                self._kill_flag = True
            if self._last_left_endpoint.pose.position.x < -0.4:
                self._kill_flag = True
        else:
            rospy.logwarn('SAFETY not RECEIVING LEFT ENDPOINT STATES')

        if self._last_right_endpoint != None:
            if self._last_right_endpoint.pose.position.z > 1.2:
                self._kill_flag = True
            if self._last_right_endpoint.pose.position.x < -0.4:
                self._kill_flag = True
        else:
            rospy.logwarn('SAFETY not RECEIVING RIGHT ENDPOINT STATES')

        if self._kill_flag:
            rospy.logerr('SAFETY VIOLATED! ENDPOINT VIOLATED')

    def _velocity_constraints(self):

        _constraint_joints = ['left_s0','left_s1','left_e0','left_e1',
            'right_s0','right_s1','right_e0','right_e1'
        ]

        if self._kill_flag:
            return

        if self._last_jointstate != None:
            for i, vel in enumerate(self._last_jointstate.velocity):
                #only make sure the bigger joints aren't moving too fast
                if self._last_jointstate.name[i] in _constraint_joints:
                    if np.abs(vel) > 1.5:
                        self._kill_flag = True
        else:
            rospy.logwarn('SAFETY not RECEIVING JOINT STATES')
        if self._kill_flag:
            rospy.logerr('SAFETY VIOLATED! VELOCITY VIOLATED')

    def _check_constraints(self):

        self._hand_endpoint_constraints()
        self._velocity_constraints()

    def spin(self):
        #our spin loop
        while not rospy.is_shutdown():
            #Step 1, check constraints
            self._check_constraints()
            #Step 2, check if kill
            if self._kill_flag:
                self.kill()

            self._spin_rate_control.sleep()

    def kill(self):
        #send the kill commands
        self._estop_pub.publish()


if __name__ == '__main__':
    rospy.init_node('safety_node')
    sn = SafetyNode()
    rospy.loginfo("Safety Node Start Running")
    sn.spin()