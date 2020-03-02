#!/usr/bin/env python
import rospy

import tf
from tf2_ros import StaticTransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import time
import pose

def update_tf(data):
	# print("pokpok")
	transform_from = "odom_wheel"
	transform_to   = "world"
	pos = data.pose.pose.position
	ori = data.pose.pose.orientation
	orientation = [ori.x, ori.y, ori.z, ori.w]
	position    = [pos.x, pos.y, pos.z]
	br = tf.TransformBroadcaster()
	br.sendTransform(position, orientation, rospy.Time.now(), transform_from, transform_to)

if __name__ == '__main__':
	rospy.init_node('static_tf_broadcaster_wheel')
	leaderPose = rospy.wait_for_message('/R3/d_odom', Odometry)

	# broadcaster.sendTransform(static_transformStamped_decawave)
	while not rospy.is_shutdown():
		# print("asas")
		update_tf(leaderPose)
	rospy.spin()
