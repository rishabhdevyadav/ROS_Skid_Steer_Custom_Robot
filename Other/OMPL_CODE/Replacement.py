#!/usr/bin/env python
######################################################################
# IIIT Hyderabad Software Distribution License
#
# Copyright (c) 2017, IIIT-H
# All Rights Reserved.
#
# For a full description see the file named LICENSE.
#
######################################################################
# Author: Pulkit Verma

# Paper followed for stanley controller
# https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_\
# Methods_for_Autonomous_Automobile_Path_Tracking.pdf

import sys
import numpy as np
import os
from os.path import abspath, dirname, join
from math import pi, cos, sin, atan, tan, atan2, sqrt, radians, degrees
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import cubic_spline_planner
from nav_msgs.msg import Odometry
import rospy
import tf
from geometry_msgs.msg import Twist

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

#Variable Definition for Stanley
dt = 0.1 
klm = 0.08  # control gain
Kp = 1.0  # speed propotional gain
L = .10  # [m] Wheel base of vehicle
max_steer = radians(80.0)  # [rad] max steering angle for Pibot
show_animation = True
target_speed = .1  # [m/s]

#Variable Definition for OMPL Planning  
robot_length = .25 # in mtrs
robot_width = .20 # in mtrs
dilation_length = robot_length    
dilation_width = robot_width    
def_points = [[robot_length/2 + dilation_length, robot_width/2 + dilation_width],\
              [-robot_length/2 - dilation_length, robot_width/2 + dilation_width],\
              [-robot_length/2 - dilation_length, -robot_width/2 - dilation_width],\
              [robot_length/2 + dilation_length, -robot_width/2 - dilation_width]]

# ROS Variables 
poseState = []

bound_limit = 8.00
newPoseX = []
newPoseY = []
ID = []
poseX = []
poseY = []
poseTh = []
total_poses = 0
newRobotId = None

# ****************** Stanley Functions ********************* 
# Update the Robot Location
def update(state, a, delta):
    if delta >= max_steer:
        delta = max_steer
    elif delta <= -max_steer:
        delta = -max_steer

    # state.x = state.x + state.v * cos(state.yaw) * dt
    # state.y = state.y + state.v * sin(state.yaw) * dt
    # implementing equation 10 of the paper
    # state.yaw = state.yaw + state.v / L * tan(delta) * dt
    if poseState:
        state.x = poseState[0]
        state.y = poseState[1]
        state.yaw = poseState[2]
    # state.yaw = pi_2_pi(state.yaw)
    state.v = state.v + a * dt
    state.w = delta
    return state

# Proportional control for velocity mismatch
def PIDControl(target, current):
    a = Kp * (target - current)
    return a

# Stanley Equation implementation --Refer to the paper link mentioned above
def stanley_control(state, cx, cy, cyaw, pind):
    ind, efa = calc_target_index(state, cx, cy)
    if pind >= ind:
        ind = pind
     # implementing equation 5 of the paper
    theta_e = pi_2_pi(cyaw[ind] - state.yaw)
    theta_d = atan2(klm * efa, state.v)
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
    fx = state.x + L * cos(state.yaw)
    fy = state.y + L * sin(state.yaw)
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

# find the Rotation matrix
def RotationMatrix(theta):
    a = cos(theta)
    b = sin(theta)
    return np.matrix([[a, -b, 0], [b, a, 0], [0, 0, 1]])

# check the collision with the obstacles
def isStateValid(state):
    global newPoseX
    global newPoseY
    point = Point(state.getX(), state.getY())
    TrueList = []
    for i in range(len(newPoseX)):
        polygon = Polygon([ (newPoseX[i][0], newPoseY[i][0]),\
                            (newPoseX[i][1], newPoseY[i][1]),\
                            (newPoseX[i][2], newPoseY[i][2]),\
                            (newPoseX[i][3], newPoseY[i][3])
                          ])
        if (polygon.contains(point)):
            TrueList.append(0)
        else:
            TrueList.append(1)
    return not any(x is 0 for x in TrueList) 

# shutdown ROS on interrupt
def shutdown():
    global cmd_vel_pub
    rospy.loginfo("Shutting Down Tracking")
    cmd_vel_pub.publish(Twist())
    rospy.sleep(1)

def update_pose(msg):
    global poseState
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    orientation = msg.pose.pose.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    poseState = [pos_x, pos_y, euler[2]]

# **************** starting MAIN code *********************
# get Robot poses
with open( pose_path + "/Pose.txt") as file: 
   all_poses = file.read()
poses = all_poses.split('\n')   
for i in range(len(poses)):
    if poses[i] == "":
        pass
    else:
        total_poses = total_poses + 1
        test = poses[i].split(' ')
        ID.append(test[0])
        poseX.append(test[1])
        poseY.append(test[2])
        poseTh.append(test[3])

ID = map(int, ID)
poseX = map(float, poseX)
poseY = map(float, poseY)
poseTh = map(float, poseTh)

# find the obstacles poses based on the robot locations
for i in range(total_poses-2):
    new_mat = []
    for j in range(4):
        mat = np.dot(RotationMatrix(poseTh[i]), np.transpose(np.matrix([def_points[j][0], \
                def_points[j][1], 1]))) + np.transpose(np.matrix([poseX[i], poseY[i], 0]))
        new_mat.append(mat.tolist())
    test_x = []
    test_y = []
    for k in range(len(new_mat)):
        test_x.append(new_mat[k][0:2][0][0])
        test_y.append(new_mat[k][0:2][1][0])
    newPoseX.append(test_x)
    newPoseY.append(test_y)

SE2 = ob.SE2StateSpace()
setup = og.SimpleSetup(SE2)
bounds = ob.RealVectorBounds(2)
bounds.setLow(-bound_limit)
bounds.setHigh(bound_limit)
SE2.setBounds(bounds)
# define start state
newRobotId = ID[-2]
start = ob.State(SE2)
start().setX(poseX[-2])
start().setY(poseY[-2])
start().setYaw(poseTh[-2])
# define goal state
goal = ob.State(SE2)
goal().setX(poseX[-1])
goal().setY(poseY[-1])
goal().setYaw(poseTh[-1])

print poseX,   poseX,poseTh

setup.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
# set the start & goal states
setup.setStartAndGoalStates(start, goal, .1)
# set the planner
planner = og.InformedRRTstar(setup.getSpaceInformation())
# planner = og.BITstar(setup.getSpaceInformation())
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
    print ax
    print ay
    # ax.reverse()
    # ay.reverse()
    if not setup.haveExactSolutionPath():
        print("Solution is approximate. Distance to actual goal is %g" %
            setup.getProblemDefinition().getSolutionDifference())   

# get a smoother curve using a cubic spline curve fit
cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.05)
lastIndex = len(cx) - 1
# initial state of the robot
state = State(x=cx[0], y=cy[0], yaw=0, v=0.0, w=0.0)
x = [state.x]
y = [state.y]
yaw = [state.yaw]
v = [state.v]
w = [state.w]
target_ind, mind = calc_target_index(state, cx, cy)

# MatplotLib initializations
fig = plt.figure()
ax2 = fig.add_subplot(111, aspect='equal')

# ROS initializations
rospy.init_node('tracking', anonymous=True)
rospy.Subscriber('/R'+str(newRobotId)+ '/d_odom', Odometry, update_pose)
cmd_vel_pub = rospy.Publisher('/R'+str(newRobotId)+ '/cmd_vel', Twist, queue_size=10)
rospy.on_shutdown(shutdown)
rate = rospy.Rate(10)

while (lastIndex > target_ind) and not rospy.is_shutdown():
    ai = PIDControl(target_speed, state.v)
    di, target_ind = stanley_control(state, cx, cy, cyaw, target_ind)
    state = update(state, ai, di)
    x.append(state.x)
    y.append(state.y)
    yaw.append(state.yaw)
    v.append(state.v)
    w.append(state.w)

    # publish the command velocities for the robot
    move_cmd = Twist()
    move_cmd.linear.x = state.v
    move_cmd.angular.z = state.w
    cmd_vel_pub.publish(move_cmd)

    # plot the values 
    if show_animation:
        ax2.cla()
        for i in range(total_poses-2):
            ax2.add_patch( patches.Polygon([ [newPoseX[i][0], newPoseY[i][0]],\
                                         [newPoseX[i][1], newPoseY[i][1]],\
                                         [newPoseX[i][2], newPoseY[i][2]],\
                                         [newPoseX[i][3], newPoseY[i][3]] \
                                       ], closed=True ))
        ax2.plot(cx, cy, ".r", label="course")
        ax2.plot(x, y, "-b", label="trajectory")
        ax2.plot(cx[target_ind], cy[target_ind], "xg", label="target")
        ax2.axis("equal")
        plt.title("Speed[m/s]:" + str(state.v)[:4])
        plt.pause(0.001)

# publish the command velocities for the robot
move_cmd = Twist()
print degrees(poseState[2])
print degrees(poseTh[-1])
while (abs(degrees(poseState[2]) - degrees(poseTh[-1]) ) > 2 ) and not rospy.is_shutdown():
    move_cmd.linear.x = 0
    move_cmd.angular.z = -0.25
    cmd_vel_pub.publish(move_cmd)

move_cmd = Twist()
move_cmd.linear.x = 0
move_cmd.angular.z = 0
cmd_vel_pub.publish(move_cmd)
