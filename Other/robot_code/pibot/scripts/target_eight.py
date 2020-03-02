#!/usr/bin/env python
from __future__ import division

import rospy
from geometry_msgs.msg import Twist
import time
import numpy as np
import tf
from math import sqrt, sin, cos, atan2, acos, pi
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import pose

class Eight():
	def __init__(self):
		rospy.init_node('Eight')
		self.cmd_vel_pub = rospy.Publisher('/R1/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('Robot1_odom', Odometry, self.currOdomCallback)
		self.time_start = rospy.Time.now()
		self.time_to_run = rospy.get_param('~time_to_run',5000)
		self.v_target = rospy.get_param('~vel', .2) # in mtr/sec
		self.w_target = rospy.get_param('~omega', 0)   
		self.angle_tolerance = 2 * pi/180 # in radians 
		self.counter = 0
		self.flag = True
		self.theta_prev = 0
		self.initPose()

	def initPose(self):
		self.myPose = pose.Pose()
		self.myPose.x = 0
		self.myPose.y = 0
		self.myPose.theta = 0

	def update(self):
		if self.myPose.theta == 0:
			print("hello handles ..............")
			self.myPose.theta = self.theta_prev

		time_duration = (rospy.Time.now() - self.time_start).to_sec()
		if time_duration > self.time_to_run:
			rospy.signal_shutdown('TimeOut Occurred for target definition')

		if (abs(self.myPose.theta) < self.angle_tolerance) and (self.flag == True):
			self.flag = False
			self.counter = self.counter + 1

		if (self.counter % 2):
			self.w_target = 0.1528
		else:
			self.w_target = -0.1528

		if abs(self.myPose.theta) > 10 * self.angle_tolerance:
			self.flag = True

		move_cmd = Twist()
		move_cmd.linear.x = self.v_target
		move_cmd.angular.z = self.w_target
		self.cmd_vel_pub.publish(move_cmd)
		self.theta_prev = self.myPose.theta

	def currOdomCallback(self, newPose):
		self.myPose = pose.Pose()
		pos = newPose.pose.pose.position
		orientation = newPose.pose.pose.orientation
		self.myPose.x = pos.x
		self.myPose.y = pos.y
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.myPose.theta = euler[2]

	def spin(self):
		rospy.loginfo("Target Defined")
		rate = rospy.Rate(10)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update()
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Shutting Down defined Target")
		self.cmd_vel_pub.publish(Twist())
		rospy.sleep(1)

def main():
	eight = Eight();
	eight.spin()

if __name__ == '__main__':
	main()