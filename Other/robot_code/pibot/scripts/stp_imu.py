#!/usr/bin/env python
import rospy

import tf
from tf2_ros import StaticTransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import time
import pose

def update_tf(data):
	transform_from = "imu"
	transform_to   = "base_link_wheel"
	ori = data.pose.pose.orientation
	quaternion = (ori.x, ori.y, ori.z, ori.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	theta = -euler[2]
	ori = Quaternion(*tf.transformations.quaternion_from_euler(0,0,theta))
	orientation = [ori.x, ori.y, ori.z, ori.w]
	print orientation
	position    = [0, 0, 0]
	br = tf.TransformBroadcaster()
	br.sendTransform(position, orientation, rospy.Time.now(), transform_from, transform_to)

if __name__ == '__main__':
	rospy.init_node('static_tf_broadcaster_imu')
	leaderPose = rospy.wait_for_message('/R3/d_odom', Odometry)

	while not rospy.is_shutdown():
		update_tf(leaderPose)
	rospy.spin()