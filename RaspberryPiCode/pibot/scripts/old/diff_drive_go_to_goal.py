#! /usr/bin/env python
from __future__ import division

import rospy
from math import pi, asin, acos
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool, Int8
import time
import goal_controller
import pose
import tf

class GoToGoal:

	def __init__(self):
		self.controller = goal_controller.GoalController()

	def main(self):
		rospy.init_node('diff_drive_go_to_goal')
		self.twistPub = rospy.Publisher('~cmd_vel', Twist, queue_size=10)
		self.stopMotor = rospy.Publisher('~goalReached', Int8, queue_size=10)
		
		rospy.Subscriber('~Robot1odom', Odometry, self.odomCallback)
		rospy.Subscriber('~move_base_simple/goal', PoseStamped, self.goalCallback)

		self.rate = rospy.get_param('~rate', 10.0)
		self.dT = 1 / self.rate
		self.linearTolerance = rospy.get_param('~linear_tolerance', .1)
		self.angularTolerance = rospy.get_param('~angular_tolerance', 10/180*pi)
		
		self.controller.setLinearTolerance(self.linearTolerance)
		self.controller.setAngularTolerance(self.angularTolerance)

		self.track_started = 0

		self.initPose()
		self.goal = None
		self.prev_pose = None
		

	def initPose(self):
		self.pose = pose.Pose()
		self.prev_pose = pose.Pose()
		self.prev_pose.x = 0
		self.prev_pose.y = 0
		self.prev_pose.theta = 0
		self.pose.x = 0
		self.pose.y = 0
		self.pose.theta = 0

	def update(self):
		if self.controller.atGoal(self.pose, self.goal):
			desired = pose.Pose()
			# if self.track_started:
			rospy.loginfo("Goal Reached")
			self.stopMotor.publish(1)
		else:
			desired = self.controller.getVelocity(self.pose, self.goal, self.prev_pose)
			self.track_started = 1

		self.prev_pose = self.pose
		twist = Twist()
		twist.linear.x = desired.xVel
		twist.angular.z = desired.thetaVel
		self.twistPub.publish(twist)

	def odomCallback(self, newPose):
		self.pose = pose.Pose()
		pos = newPose.pose.pose.position
		orientation = newPose.pose.pose.orientation
		self.pose.x = pos.x
		self.pose.y = pos.y
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.pose.theta = euler[2]

	def goalCallback(self, goal):
		self.goal = pose.Pose()
		pos = goal.pose.position
		orientation = goal.pose.orientation
		self.goal.x = pos.x
		self.goal.y = pos.y
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.goal.theta = euler[2]
		# self.goal.theta = 2 * asin(orientation.w)

	def spin(self):
		rospy.loginfo("Goal Point Set")
		rate = rospy.Rate(self.rate)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update()
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Goal Interrupted")
		self.twistPub.publish(Twist())
		rospy.sleep(1)

if __name__ == '__main__':
	gotoGoal = GoToGoal()
	gotoGoal.main()
	gotoGoal.spin()