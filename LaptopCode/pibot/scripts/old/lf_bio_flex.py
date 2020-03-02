#!/usr/bin/env python
from __future__ import division

import rospy
import time
import numpy as np
import tf
from math import sqrt, sin, cos, atan2, acos, pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import pose

class Formation():
	def __init__(self):
		rospy.init_node('Formation')
		# Follower Subscriber and Publisher
		self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		rospy.Subscriber('d_odom', Odometry, self.currOdomCallback)
		rospy.Subscriber('lij', Float32, self.lijCallback)

		self.LeaderName = rospy.get_param('~leader',"R2") 
		self.myID = rospy.get_param('~myID',"R3") 
		# Subscribing to Information related to the Leader
		rospy.Subscriber('/'+self.LeaderName+'/cmd_vel', Twist, self.twistCallback)
		rospy.Subscriber('/'+self.LeaderName+'/odom' , Odometry, self.leaderOdomCallback)
		
		self.initPose()
		self.leaderOdomCallback(rospy.wait_for_message('/'+self.LeaderName+'/odom', Odometry))
		self.currOdomCallback(rospy.wait_for_message('/'+self.myID + '/f_odom', Odometry))
		self.lijCallback(rospy.wait_for_message('/'+self.myID + '/lij', Float32))
		self.desired = self.myPose
		
		# initialize elements to zero
		self.target_v = 0.0
		self.target_w = 0.0
		self.vel = 0
		self.omega = 0
		
		self.alpha_j = 0
		self.beta_j = 0

		# target to maintain
		self.d = 0.1 # in meters
		self.phi_ijd = rospy.get_param('~phi_ijd',(4*pi)/3)

		self.phi_ijd = np.radians(self.phi_ijd)

		self.init = 1

		self.time_start = rospy.Time.now()
		self.dt = 0.1 # init to 0.1 sec

		# gains
		self.k1 = rospy.get_param('~k1',10.0)
		self.k2 = rospy.get_param('~k2',5.0)
		self.k3 = rospy.get_param('~k3',1.0)
		self.A = rospy.get_param('~A',50.0)
		self.B = rospy.get_param('~B',5.0)
		self.D = rospy.get_param('~D',5.0)

	def initPose(self):
		self.L_ijd = 0  
		self.leaderPose = pose.Pose()
		self.myPose = pose.Pose()
		self.desired = pose.Pose()

	def calcError(self, desired, actual):
		thd = desired.theta
		tha = actual.theta
		# print("actual", actual.theta)
		mat_a = np.array([[ cos(tha), sin(tha), 0 ], [ -sin(tha), cos(tha), 0 ],[ 0, 0, 1 ]])
		mat_b = np.array([[desired.x - actual.x], [desired.y - actual.y], \
						  [desired.theta - actual.theta]])
		mat_c = np.dot(mat_a, mat_b)
		return (mat_c[0], mat_c[1], mat_c[2])

	def update(self):
		if self.init == 1:
			(self.X_je, self.Y_je, self.theta_je) = self.calcError(\
													self.desired, self.myPose)
			self.init = 0
			print("start")
			print(self.X_je)
			print(self.Y_je)
			print(self.theta_je)

		Lijx = self.leaderPose.x - self.myPose.x - self.d * cos(self.leaderPose.theta)
		Lijy = self.leaderPose.y - self.myPose.y - self.d * sin(self.leaderPose.theta)
		Lij = sqrt( (Lijx) ** 2 + (Lijy) ** 2)
		phi_ij = atan2(Lijy, Lijx) - self.leaderPose.theta + pi

		theta_ij = self.leaderPose.theta - self.myPose.theta

		f1 = max(self.k1 * self.X_je, 0)
		f2 = max(self.k2 * self.Y_je, 0)
		g1 = max(-self.k1 * self.X_je, 0)
		g2 = max(-self.k2 * self.Y_je, 0)

		if self.target_v == 0 and self.target_w == 0:
			# Do not move if Leader is not moving
			self.cmd_vel_pub.publish(Twist())

		else:
			self.dt = (rospy.Time.now() - self.time_start).to_sec()
		
			term_alpha = -self.A * self.alpha_j + (self.B - self.alpha_j) * f1 - \
					  (self.D + self.alpha_j) * g1
			term_beta = -self.A * self.beta_j + (self.B - self.beta_j) * f2 - \
					  (self.D + self.beta_j) * g2

			self.alpha_j = self.alpha_j + term_alpha * self.dt
			self.beta_j = self.beta_j + term_beta * self.dt

			self.time_start = rospy.Time.now()

			self.vel = self.k1 * self.alpha_j + self.target_v * cos(theta_ij) - \
					   self.L_ijd * self.target_w * sin(self.phi_ijd + theta_ij)

			self.omega = ( self.target_v * sin(theta_ij) + self.L_ijd * self.target_w * \
						   cos(self.phi_ijd + theta_ij) + self.k2 * self.beta_j
						    + self.k3 * \
						   self.theta_je ) / self.d

			if self.target_v < 0:
				self.omega = -self.omega
			# ( self.vel, self.omega ) = self.limitVelocities(self.vel, self.omega,1, .1)

			move_cmd = Twist()
			move_cmd.linear.x = self.vel
			move_cmd.angular.z = self.omega
			self.cmd_vel_pub.publish(move_cmd)

			self.X_je = self.L_ijd * cos(self.phi_ijd + theta_ij) - Lij * cos(phi_ij + theta_ij)
			self.Y_je = self.L_ijd * sin(self.phi_ijd + theta_ij) - Lij * sin(phi_ij + theta_ij)
			self.theta_je = self.leaderPose.theta - self.myPose.theta
			if self.theta_je < -3.14:
				self.theta_je  = self.theta_je + 6.28
			if self.theta_je > 3.14:
				self.theta_je  = self.theta_je - 6.28
			# self.test_pub1.publish(self.theta_je)
			# self.desired.x = self.myPose.x
			# self.desired.y = self.myPose.y
			# self.desired.theta = self.myPose.theta
			
			

	def limitVelocities(self, vel, omega, v_range, w_range):
		if vel < -v_range:
			vel = -v_range
		if vel > v_range:
			vel = v_range
		if omega < -w_range:
			omega = -w_range
		if omega > w_range:
			omega = w_range
		return (vel, omega)

	def twistCallback(self,msg):	
		self.target_v = msg.linear.x;
		self.target_w = msg.angular.z;

	def lijCallback(self,msg):	
		self.L_ijd = msg.data;

	def leaderOdomCallback(self, newPose):
		self.leaderPose = pose.Pose()
		pos = newPose.pose.pose.position
		orientation = newPose.pose.pose.orientation
		self.leaderPose.x = pos.x
		self.leaderPose.y = pos.y
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.leaderPose.theta = euler[2]
		# print(self.leaderPose.theta)
		

	def currOdomCallback(self, newPose):
		self.myPose = pose.Pose()
		pos = newPose.pose.pose.position
		orientation = newPose.pose.pose.orientation
		self.myPose.x = pos.x
		self.myPose.y = pos.y
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		# print(quaternion)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.myPose.theta = euler[2]
		# print("new_theta ..... ", self.myPose.theta / pi * 180)

	def spin(self):
		rospy.loginfo("Follower Ready")
		rate = rospy.Rate(10)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update()
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Shutting Down Follower")
		self.cmd_vel_pub.publish(Twist())
		rospy.sleep(1)

def main():
	formation = Formation();
	formation.spin()

if __name__ == '__main__':
	main()
