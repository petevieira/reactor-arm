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
		goal.trajectory.joint_names = ["shoulder_yaw", "shoulder_pitch", "elbow_pitch", "wrist_pitch", "wrist_roll", "gripper_joint"]
		point = JointTrajectoryPoint()
		point.positions = angles
		# point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		# point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		point.time_from_start = rospy.Time.now()
		goal.trajectory.points.append(point)

		# point2 = JointTrajectoryPoint()
		# point2.positions = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		# point2.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		# point2.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		# point2.time_from_start = rospy.Duration(2.0)
		# goal.trajectory.points.append(point2)

		return goal

	def get_state(self):
		return self.traj_client.get_state()

# Main
if __name__ == '__main__':
	angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	for i in range(1, len(sys.argv) - 1):
		print '', i, sys.argv[i]
		try:
			angles[i-1] = float(sys.argv[i])
		except ValueError, e:
			print 'i=', i, ', arg=', sys.argv[i]
	print 'angles: ', str(angles)

	try:
		rospy.init_node("traj", anonymous=True)

		robot = RobotArm()

		robot.start_trajectory(robot.move_arm(angles))

		# while(not robot.get_state().isDone() and rospy.ok()):
			# usleep(50000)

	except rospy.ROSInterruptException:
		print "program interrupted before completion"