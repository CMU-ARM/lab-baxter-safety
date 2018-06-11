#!/usr/bin/env python
import sys
import os
import unittest
import rospy
import baxter_interface
import yaml
import rospkg

from baxter_core_msgs.msg import (
    AssemblyState
)

# make sure we can find the package and yaml file locally
rospack = rospkg.RosPack()
path = os.path.join(rospack.get_path("lab_baxter_safety"), "parameters.yml")

# parse parameters.yml file
with open(path, 'r') as stream:
    try:
        params = yaml.load(stream)
    except yaml.YAMLError as exc:
        print(exc)

rstart  = {'right_e0': 0.200, 'right_e1': 1.000, 'right_s0': 0.6748981686853812, 'right_s1': 0.13153885256117423, 'right_w0': 0.2750, 'right_w1': -0.07861651537912745, 'right_w2': 2.8163887265576197}
lstart  = {'left_e0': 0.05675728915176031, 'left_e1': 1.4894953450367368, 'left_s0': -0.7167525231394596, 'left_s1': 0.07478156340941391, 'left_w0': -0.07401457301547121, 'left_w1': -0.15109710760671324,  		   'left_w2': 2.848602323103213}
rtest_1 = {'right_e0': 0.13268933815208820, 'right_e1': 0.03820389530341129, 'right_s0': 0.600, 'right_s1': -0.200, 'right_w0': 0.6515583396543295, 'right_w1': -0.06404369789421602, 		     	   		   'right_w2': 2.111524554524272}
ltest_1 = {'left_e0': 0.08666991451552587, 'left_e1': 0.06864564025787226, 'left_s0': -0.7156020375485456, 'left_s1': -0.4318933286403888, 'left_w0': -0.07401457301547121, 'left_w1': 1.0082088728376881, 		   'left_w2': 3.0307625416646062}
rtest_2 = {'right_e0': 0.800, 'right_e1': 0.7819467066245896, 'right_s0': -1.650, 'right_s1': 0.34131072530450457, 'right_w0': 0.3900146153198664, 'right_w1': 0.5035291936233871, 
	   'right_w2': 2.112675040115186}
ltest_2 = {'left_e0': 0.34552917247118947, 'left_e1': 0.7972865145034438, 'left_s0': 1.1451166581564614, 'left_s1': -0.09894176081860918, 'left_w0': -0.3144660615165098, 'left_w1': 0.44600491407768406, 	     'left_w2': 3.0223256473312365}
rtest_3 = {'right_e0': 0.585980660972228, 'right_e1': 0.9500, 'right_s0': 0.08168447695489828, 'right_s1': -1.3694613483847031, 'right_w0': 0.3132233295160725, 'right_w1': -0.23776702212223913, 	   	      'right_w2': 1.88142743634146}
ltest_3 = {'left_e0': 0.1062281695610649, 'left_e1': 1.0304515942620267, 'left_s0': -0.011504855909140603, 'left_s1': -1.36294193003619, 'left_w0': -0.11619904468232009, 'left_w1': 0.24351945007680942, 		   'left_w2': 2.977073214088617}

def shouldSkip():
    if not rs.state().enabled:
	return True
    return False

class SafetyTests(unittest.TestCase):

    def test_1(self):
	    right.move_to_joint_positions(rtest_1)
	    rospy.sleep(1.5)
	if rs.state().enabled:
	    rs.disable()
	    rospy.logerr("Baxter failed to kill in invalid condition! Aborting...")
	    self.skipTest("Baxter failed to kill in invalid condition! Aborting...")
	rs.enable()

    def test_2(self):
	    rospy.sleep(3)
	    if shouldSkip():
	        self.skipTest("Previous test failed")
	    right.move_to_joint_positions(rtest_2)
	    rospy.sleep(1.5)
	    if rs.state().enabled:
	        rs.disable()
	        rospy.logerr("Baxter failed to kill in invalid condition! Aborting...")
	        self.skipTest("Baxter failed to kill in invalid condition! Aborting...")
        rs.enable()
        
    def test_3(self):
	    rospy.sleep(3)
	    if shouldSkip():
	        self.skipTest("Previous test failed")
	    right.move_to_joint_positions(rstart)
	    right.move_to_joint_positions(rtest_3)
	    rospy.sleep(1.5)
	    if rs.state().enabled:
	        rs.disable()
	        rospy.logerr("Baxter failed to kill in invalid condition! Aborting...")
	        self.skipTest("Baxter failed to kill in invalid condition! Aborting...")
	    rs.enable()
    
    def test_4(self):
    rospy.sleep(3)
	left.move_to_joint_positions(ltest_1)
	rospy.sleep(1.5)
	if rs.state().enabled:
	    rs.disable()
	    rospy.logerr("Baxter failed to kill in invalid condition! Aborting...")
	    self.skipTest("Baxter failed to kill in invalid condition! Aborting...")
	rs.enable()

    def test_5(self):
	rospy.sleep(3)
	if shouldSkip():
	    self.skipTest("Previous test failed")
	left.move_to_joint_positions(ltest_2)
	rospy.sleep(1.5)
	if rs.state().enabled:
	    rs.disable()
	    rospy.logerr("Baxter failed to kill in invalid condition! Aborting...")
	    self.skipTest("Baxter failed to kill in invalid condition! Aborting...")
	rs.enable()

    def test_6(self):
	rospy.sleep(3)
	if shouldSkip():
	    self.skipTest("Previous test failed")
	left.move_to_joint_positions(lstart)
	left.move_to_joint_positions(ltest_3)
	rospy.sleep(1.5)
	if rs.state().enabled:
	    rs.disable()
	    rospy.logerr("Baxter failed to kill in invalid condition! Aborting...")
	    self.skipTest("Baxter failed to kill in invalid condition! Aborting...")
	rs.enable()

if __name__ == '__main__':
    rospy.init_node("test")
    right = baxter_interface.Limb('right')
    left = baxter_interface.Limb('left')
    rs = baxter_interface.RobotEnable()
    right.move_to_joint_positions(rstart)
    left.move_to_joint_positions(lstart)
    unittest.main()
