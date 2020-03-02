#!/usr/bin/env python
#PIBOT demo

import roslib
import rospy

import time
from geometry_msgs.msg import Twist

if __name__== "__main__":
	rospy.init_node('pibot_demo')
	pub = rospy.Publisher("Leader/cmd_vel",Twist, queue_size=1)
	
while True:
#Go straigt
		print ("Moving Straight")
		for i in range(0,100):
			twist = Twist()
			twist.linear.x = 0.1
			twist.linear.y = 0
			twist.linear.z = 0
			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = 0

			pub.publish(twist)
			r = rospy.Rate(10)
			r.sleep()


		print ("Moving Fast")
	#Make a turn
		for i in range(0,50):
				twist = Twist()
				twist.linear.x = 0.2
				twist.linear.y = 0
				twist.linear.z = 0

				twist.angular.x = 0
				twist.angular.y = 0
				twist.angular.z = 0

				pub.publish(twist)
				r = rospy.Rate(10)
				r.sleep()
	#Move a little forward
		# time.sleep(1)

		print ("Moving Left")
	#Make a turn
		for i in range(0,150):
				twist = Twist()
				twist.linear.x = 0.1
				twist.linear.y = 0
				twist.linear.z = 0

				twist.angular.x = 0
				twist.angular.y = 0
				twist.angular.z = 0.05

				pub.publish(twist)
				r = rospy.Rate(10)
				r.sleep()
	#Move a little forward
		# time.sleep(1)
		print ("Moving Right")
		for i in range(0,150):

			twist = Twist()
			twist.linear.x = 0.01
			twist.linear.y = 0
			twist.linear.z = 0


			twist.angular.x = 0
			twist.angular.y = 0
			twist.angular.z = -0.05

			pub.publish(twist)
			r = rospy.Rate(10)
			r.sleep()

		# time.sleep(1)



