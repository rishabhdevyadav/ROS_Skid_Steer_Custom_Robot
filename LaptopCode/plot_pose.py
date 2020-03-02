#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from math import degrees
import pose
import tf


class GET_HEADING():
	def __init__(self):
		rospy.init_node('plot')
		rospy.Subscriber('/R3/odom', Odometry, self.cbPose)   
		# rospy.Subscriber('/R3/d_odom', Odometry, self.cbPose1)   
		# rospy.Subscriber('/imu/rpy/filtered', Vector3Stamped, self.cbHeading)   
		self.leaderPose = pose.Pose()
		# self.leaderPose1 = pose.Pose()

	def cbPose(self, newPose):
		self.leaderPose = pose.Pose()
		pos = newPose.pose.pose.position
		orientation = newPose.pose.pose.orientation
		self.leaderPose.x = pos.x
		self.leaderPose.y = pos.y
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.leaderPose.theta = euler[2]

	# def cbPose1(self, newPose):
	# 	self.leaderPose1 = pose.Pose()
	# 	pos = newPose.pose.pose.position
	# 	orientation = newPose.pose.pose.orientation
	# 	self.leaderPose1.x = pos.x
	# 	self.leaderPose1.y = pos.y
	# 	quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
	# 	euler = tf.transformations.euler_from_quaternion(quaternion)
	# 	self.leaderPose1.theta = euler[2]

	def update(self, fig):
		plt.plot(self.leaderPose.x, self.leaderPose.y, 'ro') # plot something
		# plt.plot(self.leaderPose1.x, self.leaderPose1.y, 'ro') # plot something
		fig.canvas.draw() 
		plt.pause(0.01)

	def spin(self):
		rospy.loginfo("Get heading")
		rate = rospy.Rate(10)
		rospy.on_shutdown(self.shutdown)
		plt.axis("equal")
		fig = plt.gcf()
		plt.plot([0], [0], '.')
		plt.plot([10], [10], '.')
		fig.show()
		fig.canvas.draw()

		while not rospy.is_shutdown():
			self.update(fig)
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Shutting Down Heading")
		rospy.sleep(1)

def main():
	GETHEADING = GET_HEADING();
	GETHEADING.spin()

if __name__ == '__main__':
	main()
