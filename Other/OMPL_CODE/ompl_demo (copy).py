#!/usr/bin/env python



# Author: Luis G. Torres, Mark Moll

try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))),'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
from math import sqrt, radians, degrees
from sys import argv
import argparse
import numpy as np
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
#import myFuncMulti
from os.path import abspath, dirname, join
pose_path = dirname(dirname(dirname(dirname(abspath(__file__)))))
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import math
import cubic_spline_planner
from nav_msgs.msg import Odometry
import rospy
import tf
from geometry_msgs.msg import Twist

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = w

max_steer = radians(30.0)  # [rad] max steering angle for Pibot
show_animation = True
target_speed = .10  # [m/s]

dt =0.1
Lfc = 0.2 #prev 0.2
Klm = 0.3 #prev 0.35
Kp = 4 #prev 4
L = 0.26

poseState = []

# find the Rotation matrix
def RotationMatrix(theta):
    a = np.cos(theta)
    b = np.sin(theta)
    return np.matrix([[a, -b, 0], [b, a, 0], [0, 0, 1]])


class ValidityChecker(ob.StateValidityChecker):
    def __init__(self, si):
        super(ValidityChecker, self).__init__(si)

    # Returns whether the given state's position overlaps the
    # circular obstacle
    def isValid(self, state):
        return self.clearance(state) > 0.0

    # Returns the distance from the given state's position to the
    # boundary of the circular obstacle.
    def clearance(self, state):
        global newPoseX
        global newPoseY
        point = Point(state[0], state[1])
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

        # Distance formula between two points, offset by the circle's
        # radius
#        return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5)) - 0.25
        #return myFuncMulti.myClearance(x, y)



def getPathLengthObjective(si):
    return ob.PathLengthOptimizationObjective(si)


def getThresholdPathLengthObj(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostThreshold(ob.Cost(1.51))
    return obj


class ClearanceObjective(ob.StateCostIntegralObjective):
    def __init__(self, si):
        super(ClearanceObjective, self).__init__(si, True)
        self.si_ = si

    # Our requirement is to maximize path clearance from obstacles,
    # but we want to represent the objective as a path cost
    # minimization. Therefore, we set each state's cost to be the
    # reciprocal of its clearance, so that as state clearance
    # increases, the state cost decreases.
    def stateCost(self, s):
        return ob.Cost(1 / self.si_.getStateValidityChecker().clearance(s))


def getClearanceObjective(si):
    return ClearanceObjective(si)


def getBalancedObjective1(si):
    lengthObj = ob.PathLengthOptimizationObjective(si)
    clearObj = ClearanceObjective(si)

    opt = ob.MultiOptimizationObjective(si)
    opt.addObjective(lengthObj, 5.0)
    opt.addObjective(clearObj, 1.0)

    return opt





def getPathLengthObjWithCostToGo(si):
    obj = ob.PathLengthOptimizationObjective(si)
    obj.setCostToGoHeuristic(ob.CostToGoHeuristic(ob.goalRegionCostToGo))
    return obj


# Keep these in alphabetical order and all lower case
def allocatePlanner(si, plannerType):
    if plannerType.lower() == "bfmtstar":
        return og.BFMT(si)
    elif plannerType.lower() == "bitstar":
        return og.BITstar(si)
    elif plannerType.lower() == "fmtstar":
        return og.FMT(si)
    elif plannerType.lower() == "informedrrtstar":
        return og.InformedRRTstar(si)
    elif plannerType.lower() == "prmstar":
        return og.PRMstar(si)
    elif plannerType.lower() == "rrtstar":
        return og.RRTstar(si)
    elif plannerType.lower() == "sorrtstar":
        return og.SORRTstar(si)
    else:
        OMPL_ERROR("Planner-type is not implemented in allocation function.");


# Keep these in alphabetical order and all lower case
def allocateObjective(si, objectiveType):
    if objectiveType.lower() == "pathclearance":
        return getClearanceObjective(si)
    elif objectiveType.lower() == "pathlength":
        return getPathLengthObjective(si)
    elif objectiveType.lower() == "thresholdpathlength":
        return getThresholdPathLengthObj(si)
    elif objectiveType.lower() == "weightedlengthandclearancecombo":
        return getBalancedObjective1(si)
    else:
        OMPL_ERROR("Optimization-objective is not implemented in allocation function.");



def plan(runTime, plannerType, objectiveType, fname, bound, start_pt, goal_pt):
    print "Run Time: ", runTime
    # Construct the robot state space in which we're planning. We're
    # planning in [0,1]x[0,1], a subset of R^2.
    space = ob.RealVectorStateSpace(2)

    # Set the bounds of space to be in [0,1].
    space.setBounds(bound[0], bound[1])

    # Construct a space information instance for this state space
    si = ob.SpaceInformation(space)

    # Set the object used to check which states in the space are valid
    validityChecker = ValidityChecker(si)
    si.setStateValidityChecker(validityChecker)

    si.setup()

    # Set our robot's starting state to be the bottom-left corner of
    # the environment, or (0,0).
    start = ob.State(space)
    start[0] = start_pt[0]
    start[1] = start_pt[1]

    # Set our robot's goal state to be the top-right corner of the
    # environment, or (1,1).
    goal = ob.State(space)
    goal[0] = goal_pt[0]
    goal[1] = goal_pt[1]

    # Create a problem instance
    pdef = ob.ProblemDefinition(si)

    # Set the start and goal states
    pdef.setStartAndGoalStates(start, goal)

    # Create the optimization objective specified by our command-line argument.
    # This helper function is simply a switch statement.
    pdef.setOptimizationObjective(allocateObjective(si, objectiveType))

    # Construct the optimal planner specified by our command line argument.
    # This helper function is simply a switch statement.
    optimizingPlanner = allocatePlanner(si, plannerType)

    # Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef)
    optimizingPlanner.setup()

    # attempt to solve the planning problem in the given runtime
    solved = optimizingPlanner.solve(runTime)

    if solved:
        # Output the length of the path found
        print("{0} found solution of path length {1:.4f} with an optimization objective value of {2:.4f}".format(optimizingPlanner.getName(), pdef.getSolutionPath().length(), pdef.getSolutionPath().cost(pdef.getOptimizationObjective()).value()))

        # If a filename was specified, output the path as a matrix to
        # that file for visualization
        if fname:
            with open(fname,'w') as outFile:
                outFile.write(pdef.getSolutionPath().printAsMatrix())
#        print "Subhasis", type(pdef.getSolutionPath().printAsMatrix())
        return True, pdef.getSolutionPath().printAsMatrix()
    else:
        print("No solution found.")
        return False, "No solution found"

# shutdown ROS on interrupt
def shutdown():
    global cmd_vel_pub
    rospy.loginfo("Shutting Down Tracking")
    cmd_vel_pub.publish(Twist())
    rospy.sleep(1)

def update(state, a, delta, delX, Lf):

    r = (Lf) / (2.0 * np.sin(delta))
    state.w = state.v /r # L * math.tan(delta)
    if poseState:
        state.x = poseState[0]
        state.y = poseState[1]
        state.yaw = poseState[2]        
    # state.x = state.x + state.v * math.cos(state.yaw) * dt
    # state.y = state.y + state.v * math.sin(state.yaw) * dt
    # state.yaw = state.yaw + state.w * dt # state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt

    return state

# calculate the index of the intermediate point to be tracked
def calc_target_index(state, cx, cy):
    klm = 0.3
    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    Lf = klm * state.v + 0.3 #Subhasis

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cx[ind + 1] - cx[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind

# Proportional control for velocity mismatch
def PIDControl(target, current):
    a = Kp * (target - current)
    return a


def pure_pursuit_control(state, cx, cy, pind):

    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

    if state.v < 0:  # back
        alpha = math.pi - alpha

    Lf = Klm * state.v + Lfc

    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)

    return alpha, ind, tx-state.x, Lf


def update_pose(msg):
    global poseState
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    orientation = msg.pose.pose.orientation
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    poseState = [pos_x, pos_y, euler[2]]

if __name__ == "__main__":

    bound_limit = 8.00
    newPoseX = []
    newPoseY = []
    ID = []
    poseX = []
    poseY = []
    poseTh = []
    total_poses = 0
    newRobotId = None

    def_points = [[0.4, 0.25],\
              [0.4, -0.25],\
              [-0.2,-0.25 ],\
              [-0.2,0.25  ]]

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

    newRobotId = ID[-2]

    #print newPoseX
    # Solve the planning problem
    success, path = plan(4, 'InformedRRTstar', 'PathLength', None, (0.0, 10.0), (poseX[-2],poseY[-2]), (poseX[-1],poseY[-1]))
    string = [s.strip() for s in path.splitlines()]
    # print string
    pathList = []
    for i in string:
            if i != '':
                    print i
                    pathList.append(i.split(" "))
    # print pathList
    for j in pathList:
            j[0] = float(j[0])
            j[1] = float(j[1])
    print pathList
    pathList = np.array(pathList)
    a,b = pathList.T
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(a, b, ds=0.05)

    lastIndex = len(cx) - 1
    # initial state of the robot
    state = State(x=cx[0], y=cy[0], yaw=0, v=0.0, w=0.0)
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    w = [state.w]
    target_ind = calc_target_index(state, cx, cy)

    # ROS initializations
    rospy.init_node('tracking', anonymous=True)
    rospy.Subscriber('/R'+str(newRobotId)+ '/f_odom', Odometry, update_pose)
    cmd_vel_pub = rospy.Publisher('/R'+str(newRobotId)+ '/cmd_vel', Twist, queue_size=10)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(10)

    fig = plt.figure()
    ax2 = fig.add_subplot(111, aspect='equal')


    while not rospy.is_shutdown() and sqrt( (cx[-1] - state.x)**2 + (cy[-1] - state.y)**2) > 0.1:
        # print sqrt( (cx[-1] - state.x)**2 + (cy[-1] - state.y)**2)
        ai = PIDControl(target_speed, state.v)
        di, target_ind, delX, Lf = pure_pursuit_control(state, cx, cy, target_ind)
        state = update(state, ai, di, delX, Lf)
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

        if show_animation:
            ax2.cla()
            for i in range(total_poses-2):
                ax2.add_patch( patches.Polygon([ [newPoseX[i][0], newPoseY[i][0]],\
                                             [newPoseX[i][1], newPoseY[i][1]],\
                                             [newPoseX[i][2], newPoseY[i][2]],\
                                             [newPoseX[i][3], newPoseY[i][3]] \
                                           ], closed=True ))
            ax2.plot(cx, cy, ".r", label="course")
            ax2.plot(poseX[-1], poseY[-1], "*k", label="course")
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
plt.show()

