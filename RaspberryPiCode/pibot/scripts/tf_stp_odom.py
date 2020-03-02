#!/usr/bin/env python
import rospy

import tf
from tf2_ros import StaticTransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import time
#import pose

def update_tf(data, robot_id):
	transform_from = "{}_initial_pose".format(robot_id)
	transform_to   = "world"
	pos = data.pose.pose.position
	ori = data.pose.pose.orientation
	orientation = [ori.x, ori.y, ori.z, ori.w]
	position    = [pos.x, pos.y, pos.z]
	br = tf.TransformBroadcaster()
	br.sendTransform(position, orientation, rospy.Time.now(), transform_from, transform_to)

if __name__ == '__main__':
	rospy.init_node('static_tf_broadcaster_wheel')
	robot_id = rospy.get_namespace().replace('/', '')
	leaderPose = rospy.wait_for_message('/{}/d_odom'.format(robot_id), Odometry)

	# broadcaster.sendTransform(static_transformStamped_decawave)
	while not rospy.is_shutdown():
		# print("asas")
		update_tf(leaderPose, robot_id)
		#print('hi')
	rospy.spin()
