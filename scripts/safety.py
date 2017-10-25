

import rospy
from std_msgs.msg import(
    Empty
)

from baxter_core_msgs.msg import(
    EndpointState,
)

from sensor_msgs.msg import(
    JointState
)


class SafetyNode(object):
    """
    Monitor the robot and make sure it follows certain constraints,
    if these constratins are violated, the robot shutdowns
    """

    def __init__(self):
        self._estop_pub = rospy.Publisher('/robot/set_super_stop',Empty,queue_size=2)
        self._spin_rate = 1000 
        self._spin_rate_control = rospy.Rate(self._spin_rate)
        self._kill_flag = False

        self._left_endpoint_sub = rospy.Subscriber('/robot/limb/left/endpoint_state',EndpointState,self._left_endpoint_cb,queue_size=1)
        self._right_endpoint_sub = rospy.Subscriber('/robot/limb/right/endpoint_state',EndpointState,self._right_endpoint_cb,queue_size=1)
        self._jointstate_sub = rospy.Subscriber('/robot/joint_state',JointState,self._jointstate_cb,queue_size=1)

        self._last_left_endpoint = None
        self._last_right_endpoint = None
        self._last_jointstate = None

    def _left_endpoint_cb(self, msg):
        self._last_left_endpoint = msg

    def _right_endpoint_cb(self, msg):
        self._last_right_endpoint = msg 

    def _jointstate_cb(self, msg):
        self._last_jointstate = msg

    def _check_constraints(self):
        #check constraits
        if self._last_left_endpoint != None and self._last_left_endpoint.pose.position.z > 1.2:
            self._kill_flag = True
        if self._last_right_endpoint != None and self._last_right_endpoint.pose.position.z > 1.2:
            self._kill_flag = True

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