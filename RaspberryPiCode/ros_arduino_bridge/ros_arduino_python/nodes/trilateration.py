#!/usr/bin/env python

# import the libraries
import socket
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

def getPostion(tagsLocations, TCP_IP, TCP_PORT):
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))

	odom_pub = rospy.Publisher('/deca_odom', Odometry, queue_size=10)
	child_frame_id = rospy.get_param('~child_frame_id','/base_link')
	frame_id = rospy.get_param('~frame_id','/odom1')

	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		data_x, data_y = update(tagsLocations, s)
		if data_x == 0 and data_y == 0:
			continue
		odom_msg = pub_odometry(data_x, data_y, frame_id, child_frame_id)
		print odom_msg
		odom_pub.publish(odom_msg)
		rate.sleep()
	rospy.spin()

def pub_odometry(data_x, data_y, frame_id, child_frame_id):
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = frame_id
    odom_msg.child_frame_id = child_frame_id
    odom_msg.pose.pose.position = Point(data_x, data_y, 0)
    odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,0))
    return odom_msg

def update(tagsLocations, soc):
	d_string = soc.recv(1024)
	# print d_string, "string"
	# d_string = "3.6055, 8.544, 7.28"
	a = d_string.split(",")
	d1 = float(a[1])
	d2 = float(a[0])
	d3 = float(a[2])
	# print "socket", d1, d2, d3
	if d1 > 10 or d2 > 10 or d3 > 10:
		return 0, 0
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
	rospy.init_node('custom_talker', anonymous=True)
	# collect max number of tags placed on the walls
	max_tags = rospy.get_param('~MAX_TAGS', "3")
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
	# print tagsLocations[0][0]
	TCP_PORT = rospy.get_param("~PORT","8888")
	TCP_IP = rospy.get_param("~IP", "localhost")
	TCP_PORT = int(TCP_PORT)

	try:
		# find the position of the anchor based on the ranges received from the anchor
		getPostion(tagsLocations, TCP_IP, TCP_PORT)
	except rospy.ROSInterruptException:
		pass