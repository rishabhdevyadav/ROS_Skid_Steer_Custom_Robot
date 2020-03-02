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

import sys
import numpy as np
from os.path import abspath, dirname, join
from math import pi, cos, sin, atan
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

bound_limit = 2.00

robot_length = .25 # in mtrs
robot_width = .20 # in mtrs
    
def_points = [[robot_length/2, robot_width/2],\
              [-robot_length/2, robot_width/2],\
              [-robot_length/2, -robot_width/2],\
              [robot_length/2, -robot_width/2]]

newPoseX = []
newPoseY = []

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

def RotationMatrix(theta):
    a = cos(theta)
    b = sin(theta)
    return np.matrix([[a, -b, 0], [b, a, 0], [0, 0, 1]])

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

ID = []
poseX = []
poseY = []
poseTh = []
total_poses = 0
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
if setup.solve(20):
    # print the (approximate) solution path: print states along the path
    # and controls required to get from one state to the next
    path = setup.getSolutionPath()
    # path.interpolate(); # uncomment if you want to plot the path
    data = (path.printAsMatrix()).split('\n')
    # print data
    X = []
    Y = []
    th = []
    for loop in range(len(data)-2):
        test = data[loop].split(' ')
        X.append(test[0])
        Y.append(test[1])
        th.append(test[2])
    X = map(float, X)
    Y = map(float, Y)
    th = map(float, th)

    # print("X : ", X)
    # print("Y : ", Y)
    # print("th : ", th)
    if not setup.haveExactSolutionPath():
        print("Solution is approximate. Distance to actual goal is %g" %
            setup.getProblemDefinition().getSolutionDifference())

    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111, aspect='equal')
    for i in range(total_poses-2):
        ax1.add_patch( patches.Polygon([ [newPoseX[i][0], newPoseY[i][0]],\
                                         [newPoseX[i][1], newPoseY[i][1]],\
                                         [newPoseX[i][2], newPoseY[i][2]],\
                                         [newPoseX[i][3], newPoseY[i][3]] \
                                       ], closed=True
                                       ))

    file = open(pose_path + "/WayPoints.txt", "w")
    file.write(" ".join(str(x) for x in X))
    file.write('\n')
    file.write(" ".join(str(x) for x in Y))
    file.write('\n')
    file.write(" ".join(str(x) for x in th))
    file.close()

    
    plt.plot(X, Y)
    plt.axis([-bound_limit, bound_limit, -bound_limit, bound_limit])
    plt.show()