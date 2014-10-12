#!/usr/bin/env python

import roslib
import rospy # for writing a ROS node
import sys
from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64
import math


# def state_callback(motor):
	# print motor.current_pos

def commander():
	pub = rospy.Publisher('/jnt_controller/command', Float64)
	rospy.init_node('commander', anonymous=True)
	r = rospy.Rate(10)

	# dt = 0
	pub.publish(0)
	# while not rospy.is_shutdown():
	# 	pub.publish(math.sin(dt))
	# 	dt += 0.05

	# 	r.sleep()

# Main
if __name__ == '__main__':


	# subscribe to the /motor_states/arm_port topic, which publishes
	# messages of type MotorStateList, and receive data from that topic in
	# the callback function state_callback
	# rospy.Subscriber("/jnt_controller/state", JointState, state_callback)
	commander()



	# keep node from exiting until node is stopped
	# rospy.spin()
