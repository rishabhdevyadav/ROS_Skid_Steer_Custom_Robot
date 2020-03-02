#!/usr/bin/env python
from __future__ import division

import rospy
import tf
from sensor_msgs.msg import Imu
from numpy import degrees, pi
import datetime
import sys
import time
import fusion

class ORIENTATION:
	def __init__(self):
		rospy.init_node('local_orientation')
		rospy.Subscriber('/R3/imu/data_raw', Imu, self.imu_cb)
		self.accel_data = (0, 0, 0)
		self.gyro_data = (0, 0, 0)
		self.fuse = fusion.Fusion()
		self.ts = datetime.datetime.now()

	def imu_cb(self, msg):
		self.accel_data = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z) 
		self.gyro_data = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)

	def update(self):
		delta = datetime.datetime.now() - self.ts
		self.ts = datetime.datetime.now()
		if (0 in self.accel_data) or (0 in self.gyro_data):
			pass
		else:
			self.fuse.update_nomag(self.accel_data, self.gyro_data, delta.microseconds/1000000.0)
		euler = tf.transformations.euler_from_quaternion(self.fuse.q)
		print self.fuse.heading
		
	def spin(self):
		rospy.loginfo("Local Orientation")
		rate = rospy.Rate(10)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update()
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Shutting Down Local Orientation")
		rospy.sleep(1)

def main():
	orient = ORIENTATION();
	orient.spin()

if __name__ == '__main__':
	main()