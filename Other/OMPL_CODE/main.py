#!/usr/bin/python
from __future__ import division

import sys
import os
import time
from os.path import abspath, dirname, join

from matplotlib import pyplot as plt
import matplotlib.patches as patches
from math import atan, atan2, sqrt
from numpy import radians, degrees, pi, sin, cos, tan
import numpy as np

from shapely.geometry.polygon import Polygon
from shapely.geometry import Point

from formation import Pibot, Leader, Follower
import cubic_spline_planner
from pose import Pose
import constants as C

# define OMPL Paths
ompl_app_root = dirname(dirname(dirname(abspath(__file__))))
pose_path = dirname(dirname(dirname(dirname(abspath(__file__)))))
try:
	from ompl import base as ob
	from ompl import geometric as og
	from ompl import control as oc
except:
	sys.path.insert(0, join(ompl_app_root, 'ompl/py-bindings' ) )
	from ompl import base as ob
	from ompl import geometric as og
	from ompl import control as oc

class State:
	def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0):
		self.x = x
		self.y = y
		self.yaw = yaw
		self.v = v
		self.w = w

def main():
	# start time to calculate dt
	time_start = time.time()
	counter = 0
	robots = []

	# plot variables
	# plot_leader = []
	# plot_followers = [[] for i in range(C.NO_OF_ROBOTS_IN_FORMATION)]

	# create instances for all the robots (ID = 0 as leader and others as followers initially)
	robots.append(Leader(0, Pose(C.init_poses[0][0],C.init_poses[0][1],C.init_poses[0][2])))
	for i in range(1, C.MAX_ROBOTS):
		robots.append(Follower(int(i), Pose(C.init_poses[i][0],C.init_poses[i][1],C.init_poses[i][2]), robots[0]))
	
	# Formation Graph (out of 20 robots)
	f_graph = {'l':1, 'f':range(1, 50)}

	# set Leader Velocity
	leadr = [i for i in robots if isinstance(i, Leader)]
	# robots[f_graph.get('l')].setCmdVel(1, .01)
	vt = [0.1]*C.MAX_ITER
	# wt = [0.0]*(C.MAX_ITER)
	wt = [0.05]*(C.MAX_ITER)
	# MatplotLib initializations
	fig = plt.figure()
	ax2 = fig.add_subplot(111, aspect='equal')
	# run formation in the main loop
	plt.axis([-3, 1, -2, 2]) 

	while True:
		try:
			plt.cla()
			plt.plot(zip(*C.init_poses)[0], zip(*C.init_poses)[1], 'k*')
			if counter >= C.MAX_ITER:
				break

			if counter == C.Break1 or counter == C.Break2:
				leadr[0].setCmdVel(0, 0)
			else:
				leadr[0].setCmdVel(vt[counter], wt[counter])

			counter  = counter + 1
			print counter
			# for r in range(C.NO_OF_ROBOTS_IN_FORMATION):
			for r in C.FORMATION_BOT_IDS:
				# dt = time.time() - time_start
				dt = .1
				# if r == f_graph.get('l'):
				if isinstance(robots[r], Leader): 
					robots[r].update(robots[r].v, robots[r].w, dt)
					# plot_leader.append(robots[r].pose.x)
					# plot_leader.append(robots[r].pose.y)
					# print robots[r].pose.x
				else:
					if r not in C.faulty_robot:
						robots[r].setCmdVel(C.Lij[r], C.Phij[r], dt)
						robots[r].update(robots[r].v, robots[r].w, dt)
						# plot_followers[r].append(robots[r].pose.x)
						# plot_followers[r].append(robots[r].pose.y)
					else:
						robots[r].update(0, 0, dt)

					# print robots[r].pose.y
				time_start = time.time()
				plt.plot(robots[r].pose.x, robots[r].pose.y, 'r.')	
			plt.pause(0.01)

			if counter == C.Break1:
				C.faulty_robot = [8, 3]
				time.sleep(2)
			if counter == C.Break2:
				new_bot_id = 9
				replacement(robots, C.faulty_robot[0], new_bot_id, ax2)
				goToGoal(robots, robots[C.faulty_robot[0]].pose.tolist(), C.hub_location, C.faulty_robot[0], ax2)
				del C.faulty_robot[0]
				continue
		except KeyboardInterrupt:
			print "Code ends"
			sys.exit(1)
	plt.show()

def goToGoal(robots, sp, gp, id, ax2):
	ob = getObstacles(robots, id) # get obstacles
	print ob
	poseX  = list(zip(*ob)[0])
	poseY  = list(zip(*ob)[1])
	poseTh = list(zip(*ob)[2])
	poseX.append(sp[0]); poseX.append(gp[0])
	poseY.append(sp[1]); poseY.append(gp[1])
	poseTh.append(sp[2]); poseTh.append(gp[2])
	getExactRobotLocationAsObstacles(poseX, poseY, poseTh)
	ax, ay = InformedRRTstar(poseX, poseY, poseTh)
	cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.05)
	plotRRTpath(cx, cy, ax2, animation=True)
	new_pose = tracking(cx, cy, cyaw, ax2, len(poseX)-2, animation=True)
	exit()

def replacement(robots, faulty_robot, new_bot_id, ax2):
	print "Doing Replacement"
	sp = getStartLocation(robots, new_bot_id) # start point
	gp = getGoalLocation( robots, faulty_robot ) # goal point
	ob = getObstacles(robots, new_bot_id) # get obstacles
	print ob
	poseX  = list(zip(*ob)[0])
	poseY  = list(zip(*ob)[1])
	poseTh = list(zip(*ob)[2])
	poseX.append(sp[0]); poseX.append(gp[0])
	poseY.append(sp[1]); poseY.append(gp[1])
	poseTh.append(sp[2]); poseTh.append(gp[2])
	getExactRobotLocationAsObstacles(poseX, poseY, poseTh)
	ax, ay = InformedRRTstar(poseX, poseY, poseTh)
	cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.05)
	plotRRTpath(cx, cy, ax2, animation=True)
	new_pose = tracking(cx, cy, cyaw, ax2, len(poseX)-2, animation=True)
	robots[faulty_robot], robots[new_bot_id] = Pibot.swap(robots[new_bot_id], robots[faulty_robot])
	C.FORMATION_BOT_IDS = [new_bot_id if x==faulty_robot else x for x in C.FORMATION_BOT_IDS]
	C.Lij[new_bot_id] = C.Lij[faulty_robot]
	C.Phij[new_bot_id] = C.Phij[faulty_robot]
	robots[new_bot_id].pose = Pose(new_pose.x, new_pose.y, new_pose.yaw)

def plotRRTpath(cx, cy, ax2, animation = False):
	if animation:
		ax2.plot(cx, cy, ".r", label="course")
		ax2.axis("equal")
		plt.pause(0.001)

def getExactRobotLocationAsObstacles(poseX, poseY, poseTh):
	total_poses = len(poseX)
	for i in range(total_poses-2):
		new_mat = []
		for j in range(4):
			mat = np.dot(RotationMatrix(poseTh[i]), np.transpose(np.matrix([C.def_points[j][0], \
					C.def_points[j][1], 1]))) + np.transpose(np.matrix([poseX[i], poseY[i], 0]))
			new_mat.append(mat.tolist())
		test_x = []
		test_y = []
		for k in range(len(new_mat)):
			test_x.append(new_mat[k][0:2][0][0])
			test_y.append(new_mat[k][0:2][1][0])
		C.newPoseX.append(test_x)
		C.newPoseY.append(test_y)

def getStartLocation(robots, new_bot_id):
	return robots[new_bot_id].pose.tolist()

def getGoalLocation(robots, fr):
	goal = []
	leadr = [i for i in robots if isinstance(i, Leader)]
	leader_pose = [leadr[0].pose.x, leadr[0].pose.y, leadr[0].pose.theta]
	goal.append(leader_pose[0] + C.Lij[fr] * cos(leader_pose[2] + C.Phij[fr]))
	goal.append(leader_pose[1] + C.Lij[fr] * sin(leader_pose[2] + C.Phij[fr]))
	goal.append(leader_pose[2])
	return goal

def getObstacles(robots, new_bot_id):
	obstacles = []
	for i in robots:
		if i.id  != new_bot_id:
			obs = []
			obs.append(i.pose.x)
			obs.append(i.pose.y)
			obs.append(i.pose.theta)
			obstacles.append(obs)
	return obstacles

# find the Rotation matrix
def RotationMatrix(theta):
	a = cos(theta)
	b = sin(theta)
	return np.matrix([[a, -b, 0], [b, a, 0], [0, 0, 1]])

# check the collision with the obstacles
def isStateValid(state):
	point = Point(state.getX(), state.getY())
	TrueList = []
	for i in range(len(C.newPoseX)):
		polygon = Polygon([ (C.newPoseX[i][0], C.newPoseY[i][0]),\
							(C.newPoseX[i][1], C.newPoseY[i][1]),\
							(C.newPoseX[i][2], C.newPoseY[i][2]),\
							(C.newPoseX[i][3], C.newPoseY[i][3])
						  ])
		if (polygon.contains(point)):
			TrueList.append(0)
		else:
			TrueList.append(1)
	return not any(x is 0 for x in TrueList)
 
def InformedRRTstar(poseX, poseY, poseTh):
	SE2 = ob.SE2StateSpace()
	setup = og.SimpleSetup(SE2)
	bounds = ob.RealVectorBounds(2)
	bounds.setLow(-C.bound_limit)
	bounds.setHigh(C.bound_limit)
	SE2.setBounds(bounds)

	# define start state
	start = ob.State(SE2)
	start().setX(poseX[-2])
	start().setY(poseY[-2])
	start().setYaw(poseTh[-2])
	# define goal state
	goal = ob.State(SE2)
	goal().setX(poseX[-1])
	goal().setY(poseY[-1])
	goal().setYaw(poseTh[-1])

	setup.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
	# set the start & goal states
	setup.setStartAndGoalStates(start, goal, .1)
	# set the planner
	planner = og.InformedRRTstar(setup.getSpaceInformation())
	setup.setPlanner(planner)
	# try to solve the problem
	if setup.solve(4):
		# print the (approximate) solution path: print states along the path
		# and controls required to get from one state to the next
		path = setup.getSolutionPath()
		# path.interpolate(); # uncomment if you want to plot the path
		data = (path.printAsMatrix()).split('\n')
		# print data
		X = []
		Y = []
		for loop in range(len(data)-2):
			test = data[loop].split(' ')
			X.append(test[0])
			Y.append(test[1])
		ax = map(float, X)
		ay = map(float, Y) 
		# print ax
		# print ay
		if not setup.haveExactSolutionPath():
			print("Solution is approximate. Distance to actual goal is %g" %
				setup.getProblemDefinition().getSolutionDifference())  
		return ax, ay 

# stanley
# ****************** Stanley Functions ********************* 
# Update the Robot Location
def update(state, a, delta):
	if delta >= C.stan_max_steer:
		delta = C.stan_max_steer
	elif delta <= -C.stan_max_steer:
		delta = -C.stan_max_steer

	state.x = state.x + state.v * cos(state.yaw) * C.stan_dt
	state.y = state.y + state.v * sin(state.yaw) * C.stan_dt
	state.yaw = state.yaw + state.v / C.stan_L * tan(delta) * C.stan_dt
	state.yaw = pi_2_pi(state.yaw)
	# print state.x, state.y
	state.v = state.v + a * C.stan_dt
	state.w = delta
	return state

# Proportional control for velocity mismatch
def PIDControl(target, current):
	a = C.stan_Kp * (target - current)
	return a

# Stanley Equation implementation --Refer to the paper link mentioned above
def stanley_control(state, cx, cy, cyaw, pind):
	ind, efa = calc_target_index(state, cx, cy)
	if pind >= ind:
		ind = pind
	 # implementing equation 5 of the paper
	theta_e = pi_2_pi(cyaw[ind] - state.yaw)
	theta_d = atan2(C.stan_klm * efa, state.v)
	delta = theta_e + theta_d
	return delta, ind

# limit the angles
def pi_2_pi(angle):
	while (angle > pi):
		angle = angle - 2.0 * pi
	while (angle < -pi):
		angle = angle + 2.0 * pi
	return angle

# calculate the index of the intermediate point to be tracked
def calc_target_index(state, cx, cy):
	fx = state.x + C.stan_L * cos(state.yaw)
	fy = state.y + C.stan_L * sin(state.yaw)
	# search nearest point index
	dx = [fx - icx for icx in cx]
	dy = [fy - icy for icy in cy]
	d = [sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
	mind = min(d)
	ind = d.index(mind)

	tyaw = pi_2_pi(atan2(fy - cy[ind], fx - cx[ind]) - state.yaw)
	if tyaw > 0.0:
		mind = - mind
	return ind, mind
	
# path tracking --> stanley controller
def tracking(cx, cy, cyaw, ax2, obs_count, animation=False):
	lastIndex = len(cx) - 1
	# initial state of the robot
	state = State(x=cx[0], y=cy[0], yaw=0, v=0.0, w=0.0)
	x = [state.x]
	y = [state.y]
	yaw = [state.yaw]
	v = [state.v]
	w = [state.w]
	target_ind, mind = calc_target_index(state, cx, cy)

	while (lastIndex > target_ind):
		ai = PIDControl(C.stan_target_speed, state.v)
		di, target_ind = stanley_control(state, cx, cy, cyaw, target_ind)
		state = update(state, ai, di)
		x.append(state.x)
		y.append(state.y)
		yaw.append(state.yaw)
		v.append(state.v)
		w.append(state.w)

		# plot the values 
		if animation:
			for i in range(obs_count):
				ax2.add_patch( patches.Polygon([ [C.newPoseX[i][0], C.newPoseY[i][0]],\
											 [C.newPoseX[i][1], C.newPoseY[i][1]],\
											 [C.newPoseX[i][2], C.newPoseY[i][2]],\
											 [C.newPoseX[i][3], C.newPoseY[i][3]] \
										   ], closed=True ))
			ax2.plot(state.x, state.y, "-b", label="trajectory")
			ax2.plot(cx[target_ind], cy[target_ind], "xg", label="target")
			ax2.axis("equal")
			plt.title("Speed[m/s]:" + str(state.v)[:4])
			plt.pause(0.001)
	return state

if __name__ == '__main__':
	main()