#!/usr/bin/python

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

# parse parameters.yml file
with open("parameters.yml", 'r') as stream:
    try:
        params = yaml.load(stream)
    except yaml.YAMLError as exc:
	print(exc)

_joints = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2','right_s0','right_s1','right_e0','right_e1','right_w0','right_w1','right_w2']
_coordinates = ['x', 'y', 'z']
_orientations = ['x', 'y', 'z', 'w']

class SafetyNode(object):
    # monitor the robot and make sure it follows certain constraints,
    # if these constratins are violated, the robot shuts down

    def __init__(self):
        self._estop_pub = rospy.Publisher('/robot/set_super_stop', Empty,queue_size=2)
        self._spin_rate = 10 
        self._spin_rate_control = rospy.Rate(self._spin_rate)
        self._kill_flag = False

        self._left_endpoint_sub = rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState,self. _left_endpoint_cb, queue_size=1)
        self._right_endpoint_sub = rospy.Subscriber('/robot/limb/right/endpoint_state', EndpointState,self._right_endpoint_cb, queue_size=1)
        self._jointstate_sub = rospy.Subscriber('/robot/joint_states', JointState,self._jointstate_cb, queue_size=1)

        self._last_left_endpoint = None
        self._last_right_endpoint = None
        self._last_jointstate = None

    def _left_endpoint_cb(self, msg):
        self._last_left_endpoint = msg

    def _right_endpoint_cb(self, msg):
        self._last_right_endpoint = msg 

    def _jointstate_cb(self, msg):
        self._last_jointstate = msg

    def _orientation_constraints(self):

	if self._kill_flag:
	    return

	# check if left endpoint oreintation is valid
	if self._last_left_endpoint != None:
            left_orientation = self._last_left_endpoint.pose.orientation
	    for i in _orientations:
		if params["endpoint_orientation"]["left"][i]["max"] == None:
		    continue
	        if getattr(left_orientation, i) > params["endpoint_orientation"]["left"][i]["max"]:
		    print("Left endpoint coord/orient/max", i, getattr(left_orientation, i), params["endpoint_orientation"]["left"][i]["max"])
		    self._kill_flag = True	
                if params["endpoint_orientation"]["left"][i]["min"] == None:
                    continue
                if getattr(left_orientation, i) < params["endpoint_orientation"]["left"][i]["min"]:
                    print("Left endpoint coord/orient/min", i, getattr(left_orientation, i), params["endpoint_orientation"]["left"][i]["min"])
                    self._kill_flag = True

        else:
            rospy.logwarn('SAFETY NOT RECEIVING LEFT ENDPOINT ORIENTATION')

        # check if right endpoint orientation is valid
	if self._last_right_endpoint != None:
	    right_orientation = self._last_right_endpoint.pose.orientation
            for i in _orientations:
		if params["endpoint_orientation"]["right"][i]["max"] == None:
		    continue
                if getattr(right_orientation, i) > params["endpoint_orientation"]["right"][i]["max"]:
		    print("Right endpoint coord/orient/max", i, getattr(right_orientation, i), params["endpoint_orientation"]["right"][i]["max"])
                    self._kill_flag = True	
                if params["endpoint_orientation"]["right"][i]["min"] == None:
                    continue
                if getattr(right_orientation, i) < params["endpoint_orientation"]["right"][i]["min"]:
                    print("Right endpoint coord/orient/min", i, getattr(right_orientation, i), params["endpoint_orientation"]["right"][i]["min"])
                    self._kill_flag = True 
        else:
            rospy.logwarn('SAFETY NOT RECEIVING RIGHT ENDPOINT ORIENTATION')

        if self._kill_flag:
            rospy.logerr('SAFETY VIOLATED! ORIENTATION VIOLATED')

    def _position_constraints(self):
        
        if self._kill_flag:
            return

	# check if left endpoint position is valid
        if self._last_left_endpoint != None:
	    left_position = self._last_left_endpoint.pose.position
	    for i in _coordinates:
		if params["endpoint_position"]["left"][i]["max"] == None:
		    continue
		if getattr(left_position, i) > params["endpoint_position"]["left"][i]["max"]:
		    print("Left endpoint coord/pos/max", i, getattr(left_position, i), params["endpoint_position"]["left"][i]["max"])
		    self._kill_flag = True
		if params["endpoint_position"]["left"][i]["min"] == None:
                    continue
                if getattr(left_position, i) < params["endpoint_position"]["left"][i]["min"]:
                    print("Left endpoint coord/pos/min", i, getattr(left_position, i), params["endpoint_position"]["left"][i]["min"])
                    self._kill_flag = True

        else:
            rospy.logwarn('SAFETY NOT RECEIVING LEFT ENDPOINT POSITION')

	# check if right endpoint position is valid
        if self._last_right_endpoint != None:
            right_position = self._last_right_endpoint.pose.position
            for i in _coordinates:
		if params["endpoint_position"]["right"][i]["max"] == None:
		    continue
                if getattr(right_position, i) > params["endpoint_position"]["right"][i]["max"]:
		    print("Right endpoint coord/pos/max", i, getattr(right_position, i), params["endpoint_position"]["right"][i]["max"])
                    self._kill_flag = True	
                if params["endpoint_position"]["right"][i]["min"] == None:
                    continue
                if getattr(right_position, i) < params["endpoint_position"]["right"][i]["min"]:
                    print("Right endpoint coord/pos/min", i, getattr(right_position, i), params["endpoint_position"]["right"][i]["min"])
                    self._kill_flag = True  
        else:
            rospy.logwarn('SAFETY NOT RECEIVING RIGHT ENDPOINT POSITION')

	# check if joint positions are valid
        if self._last_jointstate != None:
            jointstate = self._last_jointstate
            for i, pos in enumerate(jointstate.position):
                if jointstate.name[i] in _joints:
		    if params["joint_position"][jointstate.name[i]]["max"] == None:
			continue
                    if pos > params["joint_position"][jointstate.name[i]]["max"]:
                        print("Joint name/pos/max", jointstate.name[i], pos, params["joint_position"][jointstate.name[i]]["max"])
                        self._kill_flag = True
                    if params["joint_position"][jointstate.name[i]]["min"] == None:
                        continue
                    if pos < params["joint_position"][jointstate.name[i]]["min"]:
                        print("Joint name/pos/min", jointstate.name[i], pos, params["joint_position"][jointstate.name[i]]["min"])
                        self._kill_flag = True

        else:
            rospy.logwarn('SAFETY NOT RECEIVING JOINT POSITIONS')
       
	if self._kill_flag:
            rospy.logerr('SAFETY VIOLATED! POSITION VIOLATED')

    def _velocity_constraints(self):

        if self._kill_flag:
            return

	# check if left endpoint velocity is valid
	if self._last_left_endpoint != None:
	    left_angular = self._last_left_endpoint.twist.angular
	    for i in _coordinates:
		if params["endpoint_velocity"]["left"]["angular"][i] == None:
		    continue
	    	if np.abs(getattr(left_angular, i)) > params["endpoint_velocity"]["left"]["angular"][i]:
		    print("Left angular coord/vel/max", i, np.abs(getattr(left_angular, i)), params["endpoint_velocity"]["left"]["angular"][i])
		    self._kill_flag = True
            left_linear = self._last_left_endpoint.twist.linear
            for i in _coordinates:
		if params["endpoint_velocity"]["left"]["linear"][i] == None:
		    continue
                if np.abs(getattr(left_linear, i)) > params["endpoint_velocity"]["left"]["linear"][i]:
		    print("Left linear coord/vel/max", i, np.abs(getattr(left_linear, i)), params["endpoint_velocity"]["left"]["linear"][i])
                    self._kill_flag = True
	else:
            rospy.logwarn('SAFETY NOT RECEIVING LEFT ENDPOINT VELOCITY')
	
	# check if right endpoint velocity is valid
        if self._last_right_endpoint != None:
            right_angular = self._last_right_endpoint.twist.angular
            for i in _coordinates:
		if params["endpoint_velocity"]["right"]["angular"][i] == None:
		    continue     
		if np.abs(getattr(right_angular, i)) > params["endpoint_velocity"]["right"]["angular"][i]:
		    print("Right angular coord/vel/max", i, np.abs(getattr(right_angular, i)), params["endpoint_velocity"]["right"]["angular"][i])
                    self._kill_flag = True
            right_linear = self._last_right_endpoint.twist.linear
            for i in _coordinates:
		if params["endpoint_velocity"]["right"]["linear"][i] == None:
		    continue
                if np.abs(getattr(right_linear, i)) > params["endpoint_velocity"]["right"]["linear"][i]:
                    print("Right linear coord/vel/max", i, np.abs(getattr(right_linear, i)), params["endpoint_velocity"]["right"]["linear"][i])
		    self._kill_flag = True
	else:
            rospy.logwarn('SAFETY NOT RECEIVING RIGHT ENDPOINT VELOCITY')
		
	# check if joint velocities are valid
        if self._last_jointstate != None:
            jointstate = self._last_jointstate
            for i, vel in enumerate(jointstate.velocity):
                if jointstate.name[i] in _joints:
		    if params["joint_velocity"][jointstate.name[i]] == None:
			continue
                    if np.abs(vel) > params["joint_velocity"][jointstate.name[i]]:
			print("Joint name/vel/max", jointstate.name[i], np.abs(vel), params["joint_velocity"][jointstate.name[i]])
                        self._kill_flag = True
        else:
            rospy.logwarn('SAFETY NOT RECEIVING JOINT VELOCITIES')

        if self._kill_flag:
            rospy.logerr('SAFETY VIOLATED! VELOCITY VIOLATED')

    def _check_constraints(self):
	self._orientation_constraints()
        self._position_constraints()
        self._velocity_constraints()

    def spin(self):
        # our spin loop
        while not rospy.is_shutdown():
            # Step 1: check constraints
            self._check_constraints()
            # Step 2: check if robot should kill
            if self._kill_flag:
                self.kill()

            self._spin_rate_control.sleep()

    def kill(self):
        # send the kill commands
        self._estop_pub.publish()


if __name__ == '__main__':
    rospy.init_node('safety_node')
    _estop_pub = rospy.Publisher('/robot/set_super_stop', Empty,queue_size=2)
    try:
        sn = SafetyNode()
        rospy.loginfo("Safety Node Start Running")
        sn.spin()
    except:
        # seriously, this shouldn't be happening. estop just in case!!!
        r = rospy.Rate(10)
        rospy.logerr("UNKNOWN ERROR!!!! SAFETY MODULE HAVING EXCEPTIONS!!!!")
        while True:
            _estop_pub.publish()
            r.sleep()

