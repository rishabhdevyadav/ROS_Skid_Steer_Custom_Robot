#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import scipy.io as io

class defineTarget():
	def __init__(self):
		rospy.init_node('define_target')
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.time_start = rospy.Time.now()
		self.time_to_run = rospy.get_param('~time_to_run',5000)
		temp = io.loadmat('/home/pibot/Rahul/control.mat')		
		self.v_target = test['control'].tolist()[0] # in mtr/sec
		self.w_target = test['control'].tolist()[1]    

	def update(self):
		time_duration = (rospy.Time.now() - self.time_start).to_sec()
		if time_duration > self.time_to_run:
			rospy.signal_shutdown('TimeOut Occurred for target definition')

		move_cmd = Twist()
		for i in range(0,len(self.v_target)):
			move_cmd.linear.x = self.v_target[i]
			move_cmd.angular.z = self.w_target[i]
			self.cmd_vel_pub.publish(move_cmd)

	def spin(self):
		rospy.loginfo("Target Defined")
		rate = rospy.Rate(10)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update()
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Shutting Down defined Target")
		self.cmd_vel_pub.publish(Twist())
		rospy.sleep(1)

def main():
	define_target = defineTarget();
	define_target.spin()

if __name__ == '__main__':
	main()

