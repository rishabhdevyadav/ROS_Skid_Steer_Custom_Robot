#!/usr/bin/env python
import rospy

import tf
from tf2_ros import StaticTransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import time

def pose(data):
	robot_id = rospy.get_namespace().replace('/', '')
	transform_from = "{}_pose".format(robot_id)
	transform_to   = "world"
	pos = data.pose.pose.position
	ori = data.pose.pose.orientation
	#print(pos)
	orientation = [ori.x, ori.y, ori.z, ori.w]
	position    = [pos.x, pos.y, pos.z]
	br = tf.TransformBroadcaster()
	br.sendTransform(position, orientation, rospy.Time.now(), transform_from, transform_to)

if __name__ == '__main__':
	robot_id = rospy.get_namespace().replace('/', '')
	rospy.init_node('dynamic_tf_broadcaster')
	#leaderPose = rospy.wait_for_message('/{}/d_odom'.format(robot_id), Odometry)

	rospy.Subscriber('/{}/f_odom'.format(robot_id), Odometry, pose)
	rospy.spin()