from __future__ import division
from math import pi, sqrt, sin, cos, atan2
from pose import Pose

class GoalController:
	"""Finds linear and angular velocities necessary to drive toward
	a goal pose.
	"""

	def __init__(self):
		self.k1 = 0.1   # w and theta
		self.k2 = 0.02	# v and theta/w
		self.k3 = 0.2	# v and d
		self.OneDegree = 0.0174533
		self.linearTolerance = 0.05 # 2.5cm
		self.angularTolerance = 5/180*pi # 3 degrees
		self.theta_change = 0
		self.theta_err = 0

	def setLinearTolerance(self, tolerance):
		self.linearTolerance = tolerance

	def setAngularTolerance(self, tolerance):
		self.angularTolerance = tolerance

	def getGoalDistance(self, cur, goal):
		if goal is None:
			return 0
		diffX = cur.x - goal.x
		diffY = cur.y - goal.y
		return sqrt(diffX*diffX + diffY*diffY)

	def atGoal(self, cur, goal):
		if goal is None:
			return True
		d = self.getGoalDistance(cur, goal)
		dTh = abs(cur.theta - goal.theta)
		return d < self.linearTolerance and dTh < self.angularTolerance

	def adjust_omega(self, theta_err):
		w = self.k1 * self.theta_err
		if abs(w) > .06:
				w = .06
		return w
		# if self.theta_err > 0:
		# 	if self.theta_err > self.OneDegree * 10:
		# 		w = self.OneDegree * 10
		# 	else:
		# 		w = self.OneDegree

		# elif self.theta_err < 0:
		# 	if self.theta_err < -self.OneDegree * 10:
		# 		w = -self.OneDegree * 10
		# 	else:
		# 		w = -self.OneDegree
		# else:
		# 	w = 0

		# return w


	def getVelocity(self, cur, goal, prev):
		desired = Pose()
		d = self.getGoalDistance(cur, goal)
		self.theta_err = atan2(goal.y - cur.y, goal.x - cur.x) - self.theta_change
		
		# if self.theta_err > 3.14:
		# 	self.theta_err = self.theta_err - 6.28
		# if self.theta_err < -3.14:
		# 	self.theta_err = self.theta_err + 6.28

		desired.thetaVel = self.adjust_omega(self.theta_err)
		self.theta_change = atan2(cur.y - prev.y, cur.x - prev.x)

		if desired.thetaVel > self.OneDegree * 5:
			desired.xVel = self.k2 / abs(self.theta_err)
			if desired.xVel > 0.12:
				desired.xVel = 0.12
			if desired.xVel < 0.07:
				desired.xVel = 0.07

		elif abs(d) > self.linearTolerance:
			desired.xVel = self.k3 * d
			if desired.xVel > 0.12:
				desired.xVel = 0.12
			if desired.xVel < 0.07:
				desired.xVel = 0.07
		else:
		 	desired.xVel = 0
		
		return desired




	# def getVelocity(self, cur, goal, prev, dT):
	# 	desired = Pose()
	# 	d = self.getGoalDistance(cur, goal)
	# 	a = atan2(goal.y - cur.y, goal.x - cur.x)# - cur.theta
	# 	b = atan2(cur.y - prev.y, cur.x - prev.x)
		
	# 	desired.thetaVel = self.adjust_omega(a, b)

	# 	# if a > 0:
	# 	# 	if a > (0.0872665 * 2 ):
	# 	# 		desired.thetaVel = 0.0872665 * 2 #+ self.kB*b
	# 	# 	else:
	# 	# 		desired.thetaVel = 0.0174533 * 2

	# 	# elif a < 0:
	# 	# 	if a < (-0.0872665 * 2):
	# 	# 		desired.thetaVel = -0.0872665 * 2 #+ self.kB*
	# 	# 	else:
	# 	# 		desired.thetaVel = -0.0174533 * 2

	# 	# else:
	# 	# 	desired.thetaVel = 0

	# 	if abs(d) > self.linearTolerance:
	# 		desired.xVel = 0.2
	# 	else:
	# 	 desired.xVel = 0
			

	# def adjust_omega(self, theta_final, theta_counter):
	# 	if abs(theta_final) > 0.349066:
	# 		limit = 0.0872665
	# 	else:
	# 		limit = 0.0349066


	# 	if (theta_final > theta_counter):
	# 		if (theta_final - theta_counter) > limit:
	# 			w_rand = limit
	# 		else:
	# 			w_rand = .01745
					
	# 	elif (theta_final < theta_counter):
	# 		if (theta_final - theta_counter) < -limit:
	# 			w_rand = -limit
	# 		else:
	# 			w_rand = -.01745
				
	# 	else:
	# 		w_rand = 0

	# 	if abs(desired.thetaVel) > 1:
	# 			desired.thetaVel = 1
	# 	return w_rand