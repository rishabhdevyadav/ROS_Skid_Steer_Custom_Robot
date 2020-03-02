#!/usr/bin/env python
from __future__ import division

# import the libraries
import socket
import rospy
import tf
from scipy.optimize import minimize
from math import radians, degrees, sqrt
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray, Float64
from tf.broadcaster import TransformBroadcaster

def getPostion(broadcast):
	global theta
	dummmy_pub = rospy.Publisher('header', Float64, queue_size=10)
	imu_pub = rospy.Publisher('imu_fuse', Imu, queue_size=10)
	odom_pub = rospy.Publisher('d_odom', Odometry, queue_size=10)
	child_frame_id = rospy.get_param('~child_frame_id','r3_base_link_wheel')
	frame_id = rospy.get_param('~frame_id','world')

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		dummmy_pub.publish(degrees(theta))
		data_x, data_y = update()
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

# Mean Square Error
def mse(x, locations, distances):
	mse = 0.0
	for location, distance in zip(locations, distances):
		distance_calculated = sqrt( ( x[0] - location[0] ) ** 2 + ( x[1] - location[1] ) ** 2 )
		mse += (distance_calculated - distance)**2
	return mse / len(distances)


def update():
	global all_dist, prev_y, prev_x, tagsLocations, initial_location
	# all_dist = filter(lambda x:x != -1, all_dist)
	if len(all_dist) < 3:
		d1 = 0
		d2 = 0
		d3 = 0
		# print "No distances found"
	else:
		d1 = all_dist[0]
		d2 = all_dist[1]
		d3 = all_dist[2]
	if (d1 <= 0 or d2 <= 0 or d3 <= 0) or (d1 > 10 or d2 > 10 or d3 > 10):
		return prev_x, prev_y

	distances = [ d1, d2, d3 ] 
	result = minimize(
		mse,                         # The error function
		initial_location,            # The initial guess
		args=(tagsLocations, distances), # Additional parameters for mse
		method='L-BFGS-B',           # The optimisation algorithm
		options={
			'ftol':1e-5,         # Tolerance
			'maxiter': 1e+7      # Maximum iterations
		})
	location = result.x
	prev_x = location[0]
	prev_y = location[1]
	return location[0], location[1]

# Main code begin
if __name__ == '__main__':
	# create a init node for Ros
	rospy.init_node('Decawave_Odometry', anonymous=True)
	rospy.Subscriber('distances', Float32MultiArray, cbDistances)
	# rospy.Subscriber('imu/data', Imu, cbHeading)
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
	all_dist = []
	theta = 0
	prev_y = 0
	prev_x = 0
	initial_location = [0, 0]
	declination = radians(39)
	cbHeading(rospy.wait_for_message('imu/data', Imu))
	rospy.Subscriber('imu/data', Imu, cbHeading)
	try:
		# find the position of the anchor based on the ranges received from the anchor
		getPostion(odomBroadcaster)
	except rospy.ROSInterruptException:
		pass

	
