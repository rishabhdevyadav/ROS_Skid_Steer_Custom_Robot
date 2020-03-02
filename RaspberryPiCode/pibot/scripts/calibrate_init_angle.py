#!/usr/bin/env python
import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time
import numpy as np

class Calib:
	def __init__(self, robot_id):
		self.xs = []
		self.ys = []
		self.robot_id = robot_id
		self.cmd_vel_pub = rospy.Publisher('/{}/cmd_vel'.format(robot_id), Twist, queue_size=10)

	def calibrate(self, num_times):
		pub_data = Twist()

		pub_data.linear.x = 0.1
		for x in xrange(num_times):
			t1 = rospy.Time.now()
			self.note_pose()
			while (rospy.Time.now() - t1).to_sec() < 5.0:
				self.cmd_vel_pub.publish(pub_data)
			self.note_pose()
			pub_data.linear.x *= -1
			print(pub_data.linear)

	def note_pose(self):
		x, y = [], []
		t1 = rospy.Time.now()
		while (rospy.Time.now() - t1).to_sec() < 1.0:
			msg = rospy.wait_for_message('/{}/d_odom'.format(robot_id), Odometry)
			x.append(msg.pose.pose.position.x)
			y.append(msg.pose.pose.position.y)
		self.xs.append(np.mean(x))
		self.ys.append(np.mean(y))

	def compute_theta(self):
		print(np.array([self.xs, self.ys]))


# def update_tf():



if __name__ == '__main__':
	rospy.init_node('calibrate_init_imu')
	robot_id = rospy.get_namespace().replace('/', '')
	calib = Calib(robot_id)

	# leaderPose = rospy.Subscribe('/{}/d_odom'.format(robot_id), Odometry, calib.calibrate)

	calib.calibrate(10)
	calib.compute_theta()
	# while not rospy.is_shutdown():

	rospy.spin()