#!/usr/bin/env python
from __future__ import division

# import the libraries
import itertools
import socket
import rospy
import tf
import numpy as np
from math import radians, degrees
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray, Float64
from tf.broadcaster import TransformBroadcaster

def getPostion(tagsLocations, broadcast):
	global theta
	dummmy_pub = rospy.Publisher('header', Float64, queue_size=10)
	imu_pub = rospy.Publisher('imu_fuse', Imu, queue_size=10)
	odom_pub = rospy.Publisher('d_odom', Odometry, queue_size=10)
	child_frame_id = rospy.get_param('~child_frame_id','r3_base_link_wheel')
	frame_id = rospy.get_param('~frame_id','world')

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		dummmy_pub.publish(degrees(theta))
		data_x, data_y = update(tagsLocations)
		if data_x == 0 and data_y == 0:
			continue
		odom_msg = pub_odometry(broadcast, data_x, data_y, frame_id, child_frame_id)
		imu_msg = pub_imu(broadcast)
		odom_pub.publish(odom_msg)
		imu_pub.publish(imu_msg)
		rate.sleep()
	rospy.spin()

def pub_odometry(broadcast, data_x, data_y, frame_id, child_frame_id):
	global theta

	# broadcast.sendTransform(
 #                (data_x, data_y, 0), 
 #                (0, 0, 0, 1),
 #                rospy.Time.now(),
 #                child_frame_id,
 #                frame_id
 #                )

	odom_msg = Odometry()
	odom_msg.header.stamp = rospy.Time.now()
	odom_msg.header.frame_id = frame_id
	odom_msg.child_frame_id = child_frame_id#child_frame_id
	odom_msg.pose.pose.position = Point(data_x, data_y, 0)
	odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,theta))
	odom_msg.pose.covariance = 		[ 1.0 ,     0,    0,    0,    0,    0, \
										0,    1.0,    0,    0,    0,    0, \
										0,     0,  1e6,    0,    0,    0, \
										0,     0,    0,  1e6,    0,    0, \
										0,     0,    0,    0,  1e6,    0, \
										0,     0,    0,    0,    0,  1e3]
	return odom_msg

def pub_imu(broadcast):
	global theta
	imu_msg = Imu()
	imu_msg.header.stamp = rospy.get_rostime()
	imu_msg.header.frame_id = 'r3_imu_fuse'
	quaternion = Quaternion(*tf.transformations.quaternion_from_euler(0,0,theta))
	imu_msg.orientation = quaternion
	

	# broadcast.sendTransform(
 #                (0, 0, 0), 
 #                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
 #                rospy.Time.now(),
 #                'base_link_wheel',
	# 			'imu_fuse'
 #                )
	return imu_msg

def cbDistances(msg):
	global all_dist
	all_dist = msg.data

def cbHeading(msg):
	global theta, declination
	th = tf.transformations.euler_from_quaternion(
									[msg.orientation.__getattribute__(i) 
									for i in msg.orientation.__slots__])
	theta = th[2]
	# print degrees(theta), "before"
	theta = theta - declination
	if theta > radians(180):
		theta = radians(360) - theta
	if theta < -radians(180):
		theta = radians(360) + theta
	theta = -theta
	# print degrees(theta), "after"

def update(tagsLocations):
	global all_dist, prev_y, prev_x
	anchor_sequence = [0, 1, 2, 3]
	anchor_comb = list(itertools.combinations(anchor_sequence, 3))

	# all_dist = [7.21, 8.48, 7.21, 5.65]
	# all_dist = filter(lambda x:x != -1, all_dist)
	if len(all_dist) < 3:
		comb_dist = [0, 0, 0]
		return prev_x, prev_y
		# print "No distances found"
	elif len(all_dist) == 3:
		comb_dist = all_dist
		# print comb_dist, "length : {}".format(len(comb_dist))
		return prev_x, prev_y

	elif len(all_dist) == 4:
		loc_list = []
		all_dist = np.array(all_dist)
		tagsLocations = np.array(tagsLocations)
		# print tagsLocations
		for i in anchor_comb:
			try:
				comb_dist = all_dist[np.array(i)]
				tags_selected = tagsLocations[np.array(i)]
				if (np.all(comb_dist < 20) and np.all(comb_dist > 0)):
					loc_list.append(getLateration(tags_selected, comb_dist))
				else:
					return prev_x, prev_y
			except TypeError:
				print "Type Error Occurred"
				return prev_x, prev_y
				pass
		x, y = getAverage(loc_list)
		prev_x, prev_y = x, y
	else:
		comb_dist = [0, 0, 0]
		print "Some wrong sequence of distances are coming : {}".format(all_dist)
		return prev_x, prev_y
	# print x, y, "after_average"
	return x, y

def getAverage(locations):
	# print locations, "before average"
	x, y = 0, 0
	for i in locations:
		x += i[0]
		y += i[1]
	return x/4, y/4

def getLateration(tagsLocations, dist):
	d1 = dist[0]
	d2 = dist[1]
	d3 = dist[2]
	# calculate A ,B and C coifficents
	A = tagsLocations[0][0]**2 + tagsLocations[0][1]**2 - d1**2
	B = tagsLocations[1][0]**2 + tagsLocations[1][1]**2 - d2**2
	C = tagsLocations[2][0]**2 + tagsLocations[2][1]**2 - d3**2
	X32 = tagsLocations[2][0] - tagsLocations[1][0]
	X13 = tagsLocations[0][0] - tagsLocations[2][0]
	X21 = tagsLocations[1][0] - tagsLocations[0][0]

	Y32 = tagsLocations[2][1] - tagsLocations[1][1]
	Y13 = tagsLocations[0][1] - tagsLocations[2][1]
	Y21 = tagsLocations[1][1] - tagsLocations[0][1]

	x = (A * Y32 + B * Y13 + C * Y21)/(2.0*(tagsLocations[0][0]*Y32 + tagsLocations[1][0]*Y13 + tagsLocations[2][0]*Y21))
	y = (A * X32 + B * X13 + C * X21)/(2.0*(tagsLocations[0][1]*X32 + tagsLocations[1][1]*X13 + tagsLocations[2][1]*X21))
	return x, y

# Main code begin
if __name__ == '__main__':
	# create a init node for Ros
	rospy.init_node('Decawave_Odometry', anonymous=True)
	rospy.Subscriber('distances', Float32MultiArray, cbDistances)
	odomBroadcaster = TransformBroadcaster()
	# collect max number of tags placed on the walls
	max_tags = rospy.get_param('~MAX_TAGS', "4")
	# create empty list for all the tags location
	location_list = []
	# populate the list with tags location
	for i in range(int(max_tags)):
		location_list.append(rospy.get_param("~T"+str(i+1), ""))
	tagsLocations = []
	# contain all the location as floating point numbers in a list
	for i in location_list:
		tagsLocations.append(map(float, i.split(",")))
	# print tagsLocations[0], "taglocations"

	# tagsLocations = [[0,0], [10,0], [10,10], [0,10]]
	all_dist = []
	theta = 0
	prev_y = 0
	prev_x = 0
	declination = radians(39)
	cbHeading(rospy.wait_for_message('imu/data', Imu))
	rospy.Subscriber('imu/data', Imu, cbHeading)
	# declination = radians(0)
	try:
		# find the position of the anchor based on the ranges received from the anchor
		getPostion(tagsLocations, odomBroadcaster)
	except rospy.ROSInterruptException:
		pass

	
