#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped, Twist
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from math import degrees
import pose
import tf
import numpy as np
import math

class GET_HEADING():
	def __init__(self):
		rospy.init_node('plot')
		self.counter = []
		self.l1_x = []
		self.l1_y = []
		self.l2_x = []
		self.l2_y = []
		self.lwheel_x = []
		self.lwheel_y = []
		rospy.Subscriber('/R5/odometry/filtered', Odometry, self.cbPose_new)   
		rospy.Subscriber('/R5/d_odom', Odometry, self.cbPose)   
		rospy.Subscriber('/R5/odom', Odometry, self.cbPose_wheel)   
		rospy.Subscriber('/R5/cmd_vel', Twist, self.cbTwist)   
		# rospy.Subscriber('/R3/d_odom', Odometry, self.cbPose1)   
		# rospy.Subscriber('/imu/rpy/filtered', Vector3Stamped, self.cbHeading)   
		self.leaderPose = pose.Pose()
		self.leaderPose1 = pose.Pose()
		self.wheelPose = pose.Pose()
		# self.leaderPose1 = pose.Pose()

		# Estimation parameter of EKF
		self.Q = np.diag([0.001, 0.001, math.radians(20.0), 1.0])**2
		self.R = np.diag([1.0, math.radians(40.0)])**2

		#  Simulation parameter
		self.Qsim = np.diag([0.5, 0.5])**2
		self.Rsim = np.diag([1.0, math.radians(30.0)])**2

		self.DT = 0.1  # time tick [s]
		self.SIM_TIME = 50.0  # simulation time [s]
		self.cmd_vel = Twist()

	def calc_input(self):
		# v = 0.1  # [m/s]
		# yawrate = 0.1  # [rad/s]
		v = self.cmd_vel.linear.x
		yawrate = self.cmd_vel.angular.y
		u = np.matrix([v, yawrate]).T
		return u

	def observation(self, xTrue, xd, u):
		xTrue = self.motion_model(xTrue, u)

		# add noise to gps x-y
		zx = xTrue[0, 0] + np.random.randn() * self.Qsim[0, 0]
		zy = xTrue[1, 0] + np.random.randn() * self.Qsim[1, 1]
		z = np.matrix([zx, zy])

		# add noise to input
		ud1 = u[0, 0]# + np.random.randn() * Rsim[0, 0]
		ud2 = u[1, 0]# + np.random.randn() * Rsim[1, 1]
		ud = np.matrix([ud1, ud2]).T

		xd = self.motion_model(xd, ud)

		return xTrue, z, xd, ud    

	def motion_model(self, x, u):

		F = np.matrix([[1.0, 0, 0, 0],
					   [0, 1.0, 0, 0],
					   [0, 0, 1.0, 0],
					   [0, 0, 0, 0.0]])

		B = np.matrix([[self.DT * math.cos(x[2, 0]), 0],
					   [self.DT * math.sin(x[2, 0]), 0],
					   [0.0, self.DT],
					   [1.0, 0.0]])

		x = F * x + B * u

		return x

	def observation_model(self, x):
		#  Observation Model
		H = np.matrix([
			[1, 0, 0, 0],
			[0, 1, 0, 0]
		])

		z = H * x

		return z

	def jacobF(self, x, u):
		"""
		Jacobian of Motion Model

		motion model
		x_{t+1} = x_t+v*dt*cos(yaw)
		y_{t+1} = y_t+v*dt*sin(yaw)
		yaw_{t+1} = yaw_t+omega*dt
		v_{t+1} = v{t}
		so
		dx/dyaw = -v*dt*sin(yaw)
		dx/dv = dt*cos(yaw)
		dy/dyaw = v*dt*cos(yaw)
		dy/dv = dt*sin(yaw)
		"""
		yaw = x[2, 0]
		v = u[0, 0]
		jF = np.matrix([
			[1.0, 0.0, -self.DT * v * math.sin(yaw), self.DT * math.cos(yaw)],
			[0.0, 1.0, self.DT * v * math.cos(yaw), self.DT * math.sin(yaw)],
			[0.0, 0.0, 1.0, 0.0],
			[0.0, 0.0, 0.0, 1.0]])

		return jF

	def jacobH(self, x):
		# Jacobian of Observation Model
		jH = np.matrix([
			[1, 0, 0, 0],
			[0, 1, 0, 0]
		])

		return jH


	def ekf_estimation(self, xEst, PEst, z, u):

		#  Predict
		xPred = self.motion_model(xEst, u)
		jF = self.jacobF(xPred, u)
		PPred = jF * PEst * jF.T + self.Q

		#  Update
		jH = self.jacobH(xPred)
		zPred = self.observation_model(xPred)
		y = z.T - zPred
		S = jH * PPred * jH.T + self.R
		K = PPred * jH.T * np.linalg.inv(S)
		xEst = xPred + K * y
		PEst = (np.eye(len(xEst)) - K * jH) * PPred

		return xEst, PEst

	def cbTwist(self, velocities):
		self.cmd_vel = velocities

	def cbPose(self, newPose):
		self.leaderPose = pose.Pose()
		pos = newPose.pose.pose.position
		orientation = newPose.pose.pose.orientation
		self.leaderPose.x = pos.x
		self.leaderPose.y = pos.y
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.leaderPose.theta = euler[2]

	def cbPose_wheel(self, newPose):
		self.wheelPose = pose.Pose()
		pos = newPose.pose.pose.position
		orientation = newPose.pose.pose.orientation
		self.wheelPose.x = pos.x
		self.wheelPose.y = pos.y
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.wheelPose.theta = euler[2]

	def cbPose_new(self, newPose):
		self.leaderPose1 = pose.Pose()
		pos = newPose.pose.pose.position
		orientation = newPose.pose.pose.orientation
		self.leaderPose1.x = pos.x
		self.leaderPose1.y = pos.y
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		self.leaderPose1.theta = euler[2]

	def update(self, fig):
		# self.counter += 1
		# u = self.calc_input()
		# self.xTrue, z, self.xDR, ud = self.observation(self.xTrue, self.xDR, u)
		# z = np.matrix([self.leaderPose.x, self.leaderPose.y])

		# self.xEst, self.PEst = self.ekf_estimation(self.xEst, self.PEst, z, ud)

		# # store data history
		# self.hxEst = np.hstack((self.hxEst, self.xEst))
		# self.hxDR = np.hstack((self.hxDR, self.xDR))
		# self.hxTrue = np.hstack((self.hxTrue, self.xTrue))
		# self.hz = np.vstack((self.hz, z))
		# # print self.hz.shape
		# if self.hxEst.shape[1] > 100:
		# 	self.hxEst = self.hxEst[:,1:]
		# 	self.hxDR = self.hxDR[:,1:] 
		# 	self.hxTrue = self.hxTrue[:,1:]
		# 	self.hz = self.hz[1:]

		# self.counter.append([self.leaderPose.x, self.leaderPose.y])
		# plt.plot(np.array(self.hxEst[0, :]).flatten(), np.array(self.hxEst[1, :]).flatten(), ".k")

		# plot_covariance_ellipse(self.xEst, self.PEst)
		
		self.l1_x.append(self.leaderPose.x)
		self.l1_y.append(self.leaderPose.y)
		self.l2_x.append(self.leaderPose1.x)
		self.l2_y.append(self.leaderPose1.y)
		self.lwheel_x.append(self.wheelPose.x)
		self.lwheel_y.append(self.wheelPose.y)
		plt.cla()
		plt.plot(self.l1_x, self.l1_y, 'r.')
		plt.plot(self.l2_x, self.l2_y, 'b.')
		plt.plot(self.lwheel_x, self.lwheel_y, 'k.')
		# plt.plot(self.leaderPose.x, self.leaderPose.y, 'r.') # plot something
		# plt.plot(self.leaderPose1.x, self.leaderPose1.y, 'b.') # plot something
		# with open ('data_plot.txt', 'a+') as f:
			# f.write("{}, {}\n".format(self.leaderPose.x, self.leaderPose.y))	
		# print (self.counter)
		# fig.canvas.draw() 
		plt.axis("equal")
		plt.grid(True)

		plt.pause(0.01)

	def spin(self):
		rospy.loginfo("Get heading")
		rate = rospy.Rate(10)
		rospy.on_shutdown(self.shutdown)
		plt.axis("equal")
		fig = plt.gcf()
		plt.plot([0], [0], '.')
		plt.plot([8], [4.5], '.')
		fig.show()
		# fig.canvas.draw()

		self.xEst = np.matrix([[self.leaderPose.x], [self.leaderPose.y], [0.1], [0.1]])
		self.xTrue = np.matrix([[self.leaderPose.x], [self.leaderPose.y], [0.1], [0.1]])
		self.PEst = np.eye(4)

		self.xDR = np.matrix(np.zeros((4, 1)))  # Dead reckoning

		# history
		self.hxEst = self.xEst
		self.hxTrue = self.xTrue
		self.hxDR = self.xTrue
		self.hz = np.zeros((1, 2))

		while not rospy.is_shutdown():
			self.update(fig)
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		print len(self.counter)
		np.savetxt('data_plot_np.txt', self.counter)
		rospy.loginfo("Shutting Down Heading")
		rospy.sleep(1)

def main():
	GETHEADING = GET_HEADING();
	GETHEADING.spin()

if __name__ == '__main__':
	main()
