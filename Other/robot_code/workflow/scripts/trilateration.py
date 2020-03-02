#!/usr/bin/env python
from __future__ import division

# import the libraries
import socket
import rospy
import tf
import math
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

def circle_intersection(circle1, circle2):
	# return self.circle_intersection_sympy(circle1,circle2)
	x1,y1,r1 = circle1
	x2,y2,r2 = circle2
	# http://stackoverflow.com/a/3349134/798588
	dx,dy = x2-x1,y2-y1
	d = math.sqrt(dx*dx+dy*dy)
	if d > r1+r2:
		#print "#1"
		return None # no solutions, the circles are separate
	if d < abs(r1-r2):
		#print "#2"
		return None # no solutions because one circle is contained within the other
	if d == 0 and r1 == r2:
		#print "#3"
		return None # circles are coincident and there are an infinite number of solutions

	a = (r1*r1-r2*r2+d*d)/(2*d)
#print 'r1', r1, 'a', a
	h = math.sqrt(round(r1, 2)**2 - round(a,2)**2)
	xm = x1 + a*dx/d
	ym = y1 + a*dy/d
	xs1 = xm + h*dy/d
	xs2 = xm - h*dy/d
	ys1 = ym - h*dx/d
	ys2 = ym + h*dx/d

	return (xs1,ys1),(xs2,ys2)

def dist_from_circ(circle, point):
	d = math.sqrt((circle[0] - point[0])**2 + (circle[1] - point[1])**2)
	if d <= circle[2]:
		return circle[2] - d
	else:
		return d - circle[2]

def trilateration(tagsLocations, d1, d2, d3):
	# tagsLocations = [[0.0, 0.0], [10.0, 0.0], [5.0, 10.0]]

	# global all_dist, prev_y, prev_x
	# all_dist = [15.711, 15.811, 4.95]
        # all_dist = filter(lambda x:x != -1, all_dist)
        # # if len(all_dist) < 3:
        #         d1 = 0
        #         d2 = 0
        #         d3 = 0
        #         # print "No distances found"
        # else:
        #         d1 = all_dist[0]
        #         d2 = all_dist[1]
        #         d3 = all_dist[2]
        # if (d1 <= 0 or d2 <= 0 or d3 <= 0) or (d1 > 20 or d2 > 20 or d3 > 20):
        #         return prev_x, prev_y


	c1 = (tagsLocations[0][0], tagsLocations[0][1], d1)
	c2 = (tagsLocations[1][0], tagsLocations[1][1], d2)
	c3 = (tagsLocations[2][0], tagsLocations[2][1], d3)

	intersect1 = circle_intersection(c1, c2)
	intersect2 = circle_intersection(c2, c3)
	intersect3 = circle_intersection(c3, c1)

	allX, allY = [], []
	
	if intersect1 is not None:
		if dist_from_circ(c3, intersect1[0]) > dist_from_circ(c3, intersect1[1]):
			allX.append(intersect1[1][0])
			allY.append(intersect1[1][1])
		elif dist_from_circ(c3, intersect1[0]) < dist_from_circ(c3, intersect1[1]):
			allX.append(intersect1[0][0])
			allY.append(intersect1[0][1])
		else:
			allX.append(intersect1[0][0])
			allX.append(intersect1[1][0])
			allY.append(intersect1[0][1])
			allY.append(intersect1[1][1])


	if intersect2 is not None:
		if dist_from_circ(c1, intersect2[0]) > dist_from_circ(c1, intersect2[1]):
			allX.append(intersect2[1][0])
			allY.append(intersect2[1][1])
		elif dist_from_circ(c1, intersect2[0]) < dist_from_circ(c1, intersect2[1]):
			allX.append(intersect2[0][0])
			allY.append(intersect2[0][1])
		else:
			allX.append(intersect2[0][0])
			allX.append(intersect2[1][0])
			allY.append(intersect2[0][1])
			allY.append(intersect2[1][1])


	if intersect3 is not None:
		if dist_from_circ(c2, intersect3[0]) > dist_from_circ(c2, intersect3[1]):
			allX.append(intersect3[1][0])
			allY.append(intersect3[1][1])
		elif dist_from_circ(c2, intersect3[0]) < dist_from_circ(c2, intersect3[1]):
			allX.append(intersect3[0][0])
			allY.append(intersect3[0][1])
		else:
			allX.append(intersect3[0][0])
			allX.append(intersect3[1][0])
			allY.append(intersect3[0][1])
			allY.append(intersect3[1][1])


	x = float(sum(allX))/len(allX)
	y = float(sum(allY))/len(allY)

	return x, y

def update(tagsLocations):
	global all_dist, prev_y, prev_x
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
	if (d1 <= 0 or d2 <= 0 or d3 <= 0) or (d1 > 20 or d2 > 20 or d3 > 20):
		return prev_x, prev_y

	return trilateration(tagsLocations, d1, d2, d3)

	# print d1, d2, d3
	# calculate A ,B and C coifficents
	# A = tagsLocations[0][0]**2 + tagsLocations[0][1]**2 - d1**2
	# B = tagsLocations[1][0]**2 + tagsLocations[1][1]**2 - d2**2
	# C = tagsLocations[2][0]**2 + tagsLocations[2][1]**2 - d3**2
	# X32 = tagsLocations[2][0] - tagsLocations[1][0]
	# X13 = tagsLocations[0][0] - tagsLocations[2][0]
	# X21 = tagsLocations[1][0] - tagsLocations[0][0]

	# Y32 = tagsLocations[2][1] - tagsLocations[1][1]
	# Y13 = tagsLocations[0][1] - tagsLocations[2][1]
	# Y21 = tagsLocations[1][1] - tagsLocations[0][1]

	# x = (A * Y32 + B * Y13 + C * Y21)/(2.0*(tagsLocations[0][0]*Y32 + tagsLocations[1][0]*Y13 + tagsLocations[2][0]*Y21))
	# y = (A * X32 + B * X13 + C * X21)/(2.0*(tagsLocations[0][1]*X32 + tagsLocations[1][1]*X13 + tagsLocations[2][1]*X21))
	# prev_x = x
	# prev_y = y
	# # print x, y, "location"
	# return x, y

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
	declination = radians(39)
	cbHeading(rospy.wait_for_message('imu/data', Imu))
	rospy.Subscriber('imu/data', Imu, cbHeading)
	try:
		# find the position of the anchor based on the ranges received from the anchor
		getPostion(tagsLocations, odomBroadcaster)
	except rospy.ROSInterruptException:
		pass

	
