#!/usr/bin/env python
from __future__ import division

import rospy
import numpy as np
import tf
from std_msgs.msg import Float64
import time
import random

def shutdown():
    rospy.loginfo("Shutting Down Voltages")
    rospy.sleep(1)

rospy.init_node('voltage_broadcast', anonymous=True)
rospy.on_shutdown(shutdown)
rate = rospy.Rate(10)

all_voltages = [0, 0, 0, 0, 0, 0, 0, 0]

robots = np.array([1, 3, 4, 5, 6, 7, 8, 9])
rospy.Subscriber('/R1/voltage', Float64, CbR1)
rospy.Subscriber('/R3/voltage', Float64, CbR3)
rospy.Subscriber('/R4/voltage', Float64, CbR4)
rospy.Subscriber('/R5/voltage', Float64, CbR5)
rospy.Subscriber('/R6/voltage', Float64, CbR6)
rospy.Subscriber('/R7/voltage', Float64, CbR7)
rospy.Subscriber('/R8/voltage', Float64, CbR8)
rospy.Subscriber('/R9/voltage', Float64, CbR9)

while not rospy.is_shutdown():
	try:	
		print all_voltages
	except:
		print "error occurs"
		rate.sleep()
rospy.spin()

def CbR9(msg):
	global all_voltages
	all_voltages[7] = msg.data

def CbR8(msg):
	global all_voltages
	all_voltages[6] = msg.data

def CbR7(msg):
	global all_voltages
	all_voltages[5] = msg.data

def CbR6(msg):
	global all_voltages
	all_voltages[4] = msg.data

def CbR5(msg):
	global all_voltages
	all_voltages[3] = msg.data

def CbR4(msg):
	global all_voltages
	all_voltages[2] = msg.data

def CbR3(msg):
	global all_voltages
	all_voltages[1] = msg.data

def CbR1(msg):
	global all_voltages
	all_voltages[0] = msg.data