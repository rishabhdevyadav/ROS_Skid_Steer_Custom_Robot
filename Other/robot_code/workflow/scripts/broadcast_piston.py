#!/usr/bin/env python
from __future__ import division

import rospy
import numpy as np
import tf
from std_msgs.msg import Int8MultiArray
import time
import random

def shutdown():
    rospy.loginfo("Shutting Down piston broadcast")
    rospy.sleep(1)


MOVE_UP = 1
MOVE_DOWN = 2
STOP 	= 0

rospy.init_node('piston_broadcast', anonymous=True)

robot_list=[]
robot_list = rospy.get_param('~robot_list',[])
piston_status = int(rospy.get_param('~piston_status',0))
robot_list.insert(0,piston_status)

rospy.on_shutdown(shutdown)
piston_pub = rospy.Publisher('pistonListing', Int8MultiArray, queue_size=10)
rate = rospy.Rate(10)	
piston_list = Int8MultiArray()

piston_list.data = np.array(robot_list)
while not rospy.is_shutdown():
	# try:	
	piston_pub.publish(piston_list)
rospy.spin()
	# except:
	# 	print "error occurs"
	# 	piston_status = STOP
	# 	piston_list.data = np.array(robot_list)
	# 	piston_pub.publish(piston_list)
	# 	rate.sleep()
