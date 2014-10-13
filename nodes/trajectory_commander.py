#!/usr/bin/env python

import roslib
import rospy
import actionlib
import sys

import control_msgs.msg
import trajectory_msgs.msg
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

class RobotArm:
	def __init__(self):
		self.traj_client = actionlib.SimpleActionClient('/trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for trajectory action client')
		self.traj_client.wait_for_server()
		rospy.loginfo('Found trajectory action client')

	def start_trajectory(self, goal):
		goal.trajectory.header.stamp = rospy.Time.now()
		rospy.loginfo('Sending goal and waiting for result')
		self.traj_client.send_goal(goal)
		rospy.loginfo('Received result')

	def move_arm(self, angles):
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.joint_names = ["shoulder_yaw", "shoulder_pitch",
		                               "elbow_pitch", "wrist_pitch",
		                               "wrist_yaw", "gripper_joint"]
		point = JointTrajectoryPoint()
		point.positions = angles
		point.velocities = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
		# point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		point.time_from_start = rospy.Time.now()
		goal.trajectory.points.append(point)

		return goal

	def get_state(self):
		return self.traj_client.get_state()

def print_usage():
	print 'Script:  trajectory_commander.py\n'
	print 'Usage:   rosrun reactor_arm trajectory_commander.py q1 q2 q3 q4 q5 q6'
	print '         (where q is the desired joint angle in radians for that joint on the robot arm)\n'
	print 'Example: rosrun reactor_arm trajectory_commander.py 0 -1 -1 0 0 0\n'
	print 'Note:    integers and floats are both valid joint angles'

# Main
if __name__ == '__main__':
	if len(sys.argv) is not 7:
		print_usage()
		exit()

	try:
		rospy.init_node("traj", anonymous=True)

		angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		for i in range(1, len(sys.argv)):
			try:
				angles[i-1] = float(sys.argv[i])
			except ValueError, e:
				rospy.logerror('Value Error: i=', i, ', arg=', sys.argv[i])

		rospy.loginfo('Moving arm to joint angles: ' + str(angles) + ' radians')

		robot = RobotArm()

		robot.start_trajectory(robot.move_arm(angles))

	except rospy.ROSInterruptException:
		print "program interrupted before completion"