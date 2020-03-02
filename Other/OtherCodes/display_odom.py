#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import pose
import tf
import sys
import numpy as np

class DISPLAY_ODOM():
	def __init__(self):
		rospy.init_node('Display_Odom_{}'.format(sys.argv[2]))
		self.leaderPose = pose.Pose()
		rospy.Subscriber(sys.argv[1], Odometry, self.cbPose)   

	def cbPose(self, newPose):
		pos = newPose.pose.pose.position
		orientation = newPose.pose.pose.orientation
		self.leaderPose.x = pos.x
		self.leaderPose.y = pos.y
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.leaderPose.theta = euler[2]

	def update(self):
		print "{}, {}, {}".format(self.leaderPose.x, self.leaderPose.y, np.degrees(self.leaderPose.theta))

	def spin(self):
		rospy.loginfo("Closing Displaying Odom")
		rate = rospy.Rate(10)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update()
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Shutting Down Heading")
		rospy.sleep(1)

def main():
	DISPLAYODOM = DISPLAY_ODOM();
	DISPLAYODOM.spin()

if __name__ == '__main__':
	main()
