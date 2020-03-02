#!/usr/bin/env python
from __future__ import division

import rospy
from std_msgs.msg import Float64


class movePiston():
	def __init__(self):
		rospy.init_node('move_piston')
		self.robots = [2,3,4]   
		self.MOVE_UP = 50
		self.MOVE_DOWN = 9.8
		self.piston_pub = []
		for i in self.robots:
			self.piston_pub.append(rospy.Publisher('/minion_' + str(i) + '/rrbot_control/command', Float64, queue_size=10))

	def update(self):
		for i, j in enumerate(self.robots):
			self.piston_pub[i].publish(self.MOVE_UP)


	def spin(self):
		rospy.loginfo("Piston Movement")
		rate = rospy.Rate(10)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update()
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Shutting Down Piston")
		rospy.sleep(1)

def main():
	move_piston = movePiston();
	move_piston.spin()

if __name__ == '__main__':
	main()

