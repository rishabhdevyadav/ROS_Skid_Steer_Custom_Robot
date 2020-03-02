#!/usr/bin/env python
from __future__ import division

import numpy as np
from numpy import sqrt, sin, cos, pi, radians, degrees 
from numpy import arctan2 as atan2, arccos as acos
# from math import sqrt, sin, cos, atan2, acos, pi, radians, degrees
import constants as C

class Pibot(object):
	def __init__(self, id, pose, v, w, voltage):
		self.pose = pose
		self.id = id
		self.v = v
		self.w = w
		self.voltage = voltage
		self.trajectory = [self.pose.tolist()]

	def update(self, v, w, dt):
		self.v = v
		self.w = w
		self.pose.update(v, w, dt)
		self.trajectory.append(self.pose.tolist())

	@staticmethod
	def swap(i1, i2):
		if isinstance(i1, Follower) and isinstance(i2, Follower):
			return Pibot.follower2follower(i1,i2)
		elif isinstance(i2, Leader):
			return Pibot.swap(i2, i1)
		else:
			return Pibot.leader2follower(i1, i2)

	@staticmethod
	def follower2follower(i1, i2):
		i2.pose, i1.pose = i1.pose, i2.pose
		return i1, i2

	# @staticmethod
	# def follower2follower(i1, i2):
	# 	not_to_change = ['pose', 'type', 'leader', 'alpha_j', 'beta_j', 'init']
	# 	ll = {k: v for k, v in i1.__dict__.items() if k not in not_to_change}
	# 	ff = {k: v for k, v in i2.__dict__.items() if k not in not_to_change and k in ll.keys()}
	# 	for k, v in ff.items():
	# 		i1.__dict__[k] = v
	# 	for k, v in ll.items():
	# 		i2.__dict__[k] = v
	# 	return i2, i1

	@staticmethod
	def leader2follower(l, f):
		not_to_change = ['pose', 'type']
		ll = {k: v for k, v in l.__dict__.items() if k not in not_to_change}
		ff = {k: v for k, v in f.__dict__.items() if k not in not_to_change and k in ll.keys()}
		for k, v in ff.items():
			l.__dict__[k] = v
		for k, v in ll.items():
			f.__dict__[k] = v
		return f, l

	def __repr__(self):
		return self.type+"{}".format(self.id)


class Leader(Pibot):
	def __init__(self, id, pose, v=0, w=0, voltage=0):
		self.type = "L"
		super(Leader, self).__init__(id, pose, v, w, voltage)

	def setCmdVel(self, v, w):
		self.v, self.w = v, w

class Follower(Pibot):
	def __init__(self, id, pose, leader, v=0, w=0, voltage=0):
		self.leader = leader
		self.alpha_j = 0
		self.beta_j = 0
		self.init = 1
		self.type = "F"
		super(Follower, self).__init__(id, pose, v, w, voltage)

	def calcError(self, desired, actual):
		thd = desired.theta
		tha = actual.theta
		mat_a = np.array([[ cos(tha), sin(tha), 0 ], [ -sin(tha), cos(tha), 0 ],[ 0, 0, 1 ]])
		mat_b = np.array([[desired.x - actual.x], [desired.y - actual.y], \
						  [desired.theta - actual.theta]])
		# print mat_a, mat_b
		mat_c = np.dot(mat_a, mat_b)
		return (mat_c[0], mat_c[1], mat_c[2])

	def setCmdVel(self, L_ijd, phi_ijd,  dt=0.1):
		if self.init == 1:
			(self.X_je, self.Y_je, self.theta_je) = np.array([0,0,0])#self.calcError(self.pose, self.pose)
			self.init = 0
			# print("starting formation for {}".format(self.id))

		Lijx = self.leader.pose.x - self.pose.x - C.d * cos(self.leader.pose.theta)
		Lijy = self.leader.pose.y - self.pose.y - C.d * sin(self.leader.pose.theta)
		Lij = sqrt( (Lijx) ** 2 + (Lijy) ** 2)
		phi_ij = atan2(Lijy, Lijx) - self.leader.pose.theta + pi
		theta_ij = self.leader.pose.theta - self.pose.theta

		f1 = max(C.k1 * self.X_je, 0)
		f2 = max(C.k2 * self.Y_je, 0)
		g1 = max(-C.k1 * self.X_je, 0)
		g2 = max(-C.k2 * self.Y_je, 0)

		if self.leader.v == 0 and self.leader.w == 0:
			# Do not move if Leader is not moving
			return [0, 0]
		else:
					
			term_alpha = -C.A * self.alpha_j + (C.B - self.alpha_j) * f1 - \
					  (C.D + self.alpha_j) * g1
			term_beta = -C.A * self.beta_j + (C.B - self.beta_j) * f2 - \
					  (C.D + self.beta_j) * g2

			self.alpha_j = self.alpha_j + term_alpha * dt
			self.beta_j = self.beta_j + term_beta * dt

			self.v = C.k1 * self.alpha_j + self.leader.v * cos(theta_ij) - \
					   L_ijd * self.leader.w * sin(phi_ijd + theta_ij)

			self.w = ( self.leader.v * sin(theta_ij) + L_ijd * self.leader.w * \
						   cos(phi_ijd + theta_ij) + C.k2 * self.beta_j
							+ C.k3 * self.theta_je ) / C.d

			self.X_je = L_ijd * cos(phi_ijd + theta_ij) - Lij * cos(phi_ij + theta_ij)
			self.Y_je = L_ijd * sin(phi_ijd + theta_ij) - Lij * sin(phi_ij + theta_ij)
			self.theta_je = self.leader.pose.theta - self.pose.theta
			if self.theta_je < -3.14:
				self.theta_je  = self.theta_je + 6.28
			if self.theta_je > 3.14:
				self.theta_je  = self.theta_je - 6.28
			return [self.v, self.w]