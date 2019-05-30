#!/usr/bin/python

import yaml
import numpy as np
import rospy
import os
import rospkg
from std_msgs.msg import Empty
from baxter_core_msgs.msg import EndpointState
from sensor_msgs.msg import JointState
from baxter_interface import RobotEnable


class SafetyNode(object):
    # monitor the robot and make sure it follows certain constraints,
    # if these constratins are violated, the robot shuts down

    def __init__(self):
        # make sure we can find the package and yaml file locally
        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path("lab_baxter_safety"), "parameters.yml")

        # parse parameters.yml file
        with open(path, 'r') as stream:
            try:
                self._params = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                rospy.logerr(exc)

        self._estop_pub = rospy.Publisher('/robot/set_super_stop', Empty,queue_size=2)
        self._spin_rate_control = rospy.Rate(10)
        self._kill_flag = False

        self._joints = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2','right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
        self._coordinates = ['x', 'y', 'z']
        self._orientations = ['x', 'y', 'z', 'w']

        self._left_endpoint_sub = rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, self._left_endpoint_cb, queue_size=1)
        self._right_endpoint_sub = rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState, self._right_endpoint_cb, queue_size=1)
        self._jointstate_sub = rospy.Subscriber('/robot/joint_states', JointState, self._jointstate_cb, queue_size=1)

        self._last_left_endpoint = None
        self._last_right_endpoint = None
        self._last_jointstate = None
        self._constraints = ["min", "max"]

    def _left_endpoint_cb(self, msg):
        self._last_left_endpoint = msg

    def _right_endpoint_cb(self, msg):
        self._last_right_endpoint = msg 

    def _jointstate_cb(self, msg):
        self._last_jointstate = msg

    def _endpoint_constraints(self):

        if self._kill_flag:
            return

        self._endpoints = {"left": self._last_left_endpoint, "right": self._last_right_endpoint}
        for side, endpoint in self._endpoints.items():
            if endpoint:
                self._checks = {"endpoint_position": (self._coordinates, endpoint.pose.position), "endpoint_orientation": (self._orientations, endpoint.pose.orientation)}
                for check, lists in self._checks.items():
                    for i in lists[0]:
                        for constraint in self._constraints:
                            # skip empty yaml file entries
                            if not self._params[check][side][i][constraint]:
                                continue
                            # compare values to max parameters
                            actual = getattr(lists[1], i)
                            limit = self._params[check][side][i][constraint]
                            if (actual > limit and constraint == "max") or (actual < limit and constraint == "min"):
                                rospy.logerr("{} endpoint, coord: {}  {}: {}  {}: {}".format(side, i, check, getattr(lists[1], i), constraint, self._params[check][side][i][constraint]))
                                self._kill_flag = True	
                                break
            else:
                rospy.logwarn('SAFETY NOT RECEIVING ENDPOINT INFORMATION')

        if self._kill_flag:
            rospy.logerr('SAFETY VIOLATED! ENDPOINT CONSTRAINT VIOLATED')

    def _joint_constraints(self):
        
        if self._kill_flag:
            return

        if self._last_jointstate:
            jointstate = self._last_jointstate
            # check if joint positions are valid
            for i, pos in enumerate(jointstate.position):
                if jointstate.name[i] in self._joints:
                    for constraint in self._constraints:
                        if not self._params["joint_position"][jointstate.name[i]][constraint]:
                            continue
                        limit = self._params["joint_position"][jointstate.name[i]][constraint]
                        if (pos > limit and constraint == "max") or (pos < limit and constraint == "min"):
                            rospy.logerr("Joint position, name: %s  pos: %s  %s: %s", jointstate.name[i], pos, constraint, self._params["joint_position"][jointstate.name[i]]["max"])
                            self._kill_flag = True
                            break
            # check if joint velocities are valid
            for i, vel in enumerate(jointstate.velocity):
                if jointstate.name[i] in self._joints:
                    if not self._params["joint_velocity"][jointstate.name[i]]:
                        continue
                    if np.abs(vel) > self._params["joint_velocity"][jointstate.name[i]]:
                        rospy.logerr("Joint, name: %s  vel: %s  max: %s", jointstate.name[i], np.abs(vel), self._params["joint_velocity"][jointstate.name[i]])
                        self._kill_flag = True
        else:
            rospy.logwarn('SAFETY NOT RECEIVING JOINT INFORMATION')

        if self._kill_flag:
            rospy.logerr('SAFETY VIOLATED! JOINT CONSTRAINT VIOLATED')

    def _check_constraints(self):
        self._endpoint_constraints()
        self._joint_constraints()

    def spin(self):
        # Spin loop
        while not rospy.is_shutdown():
            # Step 1: Check constraints
            self._check_constraints()
            # Step 2: Check if robot should kill
            if self._kill_flag:
                self.kill()
                rs.disable()
	        # Sleep
            self._spin_rate_control.sleep()

    def kill(self):
        # Send the kill commands
        self._estop_pub.publish()


if __name__ == '__main__':
    rospy.init_node('safety_node')
    estop_pub = rospy.Publisher('/robot/set_super_stop', Empty, queue_size=2)
    try:
        sn = SafetyNode()
        rs = RobotEnable()
        rospy.loginfo("Safety Node Start Running")
        sn.spin()
    except Exception as e:
        # This shouldn't be happening. Estop just in case!!!
        r = rospy.Rate(10)
        rospy.logerr(e)
        rospy.logerr("SAFETY MODULE HAVING EXCEPTIONS!!!!")
        while not rospy.is_shutdown():
            estop_pub.publish()
            r.sleep()
