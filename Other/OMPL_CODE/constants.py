from __future__ import division
from numpy import radians, degrees, pi, sqrt, arctan2

# dt time step
dt = 0.1
d_dt = 0.004

# Max iterations
MAX_ITER = 20000

# Robot and dilation for obstacles
#Variable Definition for OMPL Planning  
robot_length = .25 # in mtrs
robot_width = .20 # in mtrs
dilation_length = robot_length/2   
dilation_width = robot_width/2    
def_points = [[robot_length/2 + dilation_length, robot_width/2 + dilation_width],\
              [-robot_length/2 - dilation_length, robot_width/2 + dilation_width],\
              [-robot_length/2 - dilation_length, -robot_width/2 - dilation_width],\
              [robot_length/2 + dilation_length, -robot_width/2 - dilation_width]]


# Robot Hardware parameters
L = 0.12 # Wheel base = 2 * L (in meters)
R = 0.035 # radius of wheels (in meters)
d = 0.1 # in meters

# Formation gains
k1 = 1.5
k2 = 1.0
k3 = 0.025
A = 10.0
B = 1.0
D = 1.0

# Proportional Control Gain
Kp = 0.1
Kd = 0.6
Ki = 0.3
PWM_LIMIT = 255

# Formation parameters for payload transport
STARTING_LOCATION = [0, 0]
MAX_ROBOTS = 10
MAX_LOAD_PER_ROBOT = 5 # in Kgs
PAYLOAD_WEIGHT = 20 # in Kgs (min 10 Kgs)
MAX_HUBS = 5
STARTING_HUB = 0
NO_OF_ROBOTS_IN_FORMATION = 9#max(5, int(PAYLOAD_WEIGHT/MAX_LOAD_PER_ROBOT) + 1)
LOAD_PER_ROBOT = min( PAYLOAD_WEIGHT/NO_OF_ROBOTS_IN_FORMATION, MAX_LOAD_PER_ROBOT)

# Robot Dynamics Parameters
Lc = 0.25 # Length of chassis (in mtrs)
Wc = 0.20 # Width of chassis (in mtrs)
Mr = 1.0# Mass of robot (in Kgs)
Mw = 0.1 # Mass of Wheel (in Kgs)
Ml = 5 # load to carry 
Mt = Mr + 2*Mw +Ml # Total mass, a robot has to carry

Ir = (Mr/12.0) * (Lc ** 2 + Wc ** 2) # Inertia of robot
Iw = Mw/2.0 * (R ** 2) # Wheel Inertia
Kt = 0.2824 # MOTOR CONSTANT
Ke = 0.7638 # MOTOR CONSTANT
Io = 0 # Motor Leakage Current in amps (ignored)
b = 0  #Constant 10^-5 (ignored)
Rm = 2.4 # Motor Resistance (in ohms)
ETA = 0.3 # Motor efficiency
g = 9.8196 # Gravitational Constant
u = .1 # Frictional Constant
ks = 1000 # Frictional Constant

# test case, to be removed if dealing with dynamics
CHARGE_RATE = 0.045
DISCHARGE_RATE = 0.12 # 150

# print "Mass : ", Ml
# my_current = Mt
# print "my_rate : ", my_current

# Battery Constants
BATTERY_VOLTAGE = 12.0
BATTERY_MAX_CAPACITY = 1200.0 # in mAH
MAX_BATTERY_STATES = 10 #(10, 20, 30, 40, 50, 60, 70, 80, 90, 100)
HUB_TO_HUB_DISCHARGE_CONSTANT = 750/MAX_HUBS
HUB_TO_HUB_CHARGE_CONSTANT 	= -HUB_TO_HUB_DISCHARGE_CONSTANT*(CHARGE_RATE/DISCHARGE_RATE)
EPSILON = 0.3
Ik = 0 # Noise Current for faster battery discharge (not necessary to use)
DTH = 1100.0/BATTERY_MAX_CAPACITY
DC = HUB_TO_HUB_DISCHARGE_CONSTANT/BATTERY_MAX_CAPACITY
CC = HUB_TO_HUB_CHARGE_CONSTANT/BATTERY_MAX_CAPACITY

# optimization Constants
ALPHA 	= 0.71
BETA 	= 1 - ALPHA

# RUN_TYPE = 'OPTIMIZE'
RUN_TYPE = 'BASELINE'
BASELINE_THRESH = 20

NOR = 3;
NOC = 3;

# Pose to maintain from leader. Leader is always the 1st robot in formation
dist = 1.0
diag_dist = sqrt(dist**2+dist**2)
s_diag_dist = sqrt((2*dist)**2 + dist**2)
# Lij = [0, dist, diag_dist, dist, diag_dist, \
		  # dist, diag_dist, dist, diag_dist]  # in meters
Lij = [0, dist, 2*dist, s_diag_dist, 2*diag_dist, \
		  s_diag_dist, 2*dist, dist, diag_dist, 0]  # in meters

# Phij = []
# phi_eq = radians(360/(NO_OF_ROBOTS_IN_FORMATION-1)) # angle to maintain from leader, eg: 5 robots in formation, 1 leader and 4 followers in at 90 degrees each (360/4)
# Phij.append(radians(0)) # leader Orientation
# for i in range(1, NO_OF_ROBOTS_IN_FORMATION):
# 	phi = Phij[0] + (i-1) * phi_eq
# 	if phi > pi:
# 		phi = phi - 2*pi
# 	if phi < -pi:
# 		phi = phi + 2*pi
# 	Phij.append(phi)

Phij = [0, radians(-180), radians(-180), arctan2(-dist, -2*dist), radians(-135),\
 		arctan2(-2*dist, -dist), radians(-90), radians(-90), radians(-135), 0]

init_poses = []
x = 0
y = 0
init_poses.append([x, y, radians(90)])
init_poses.append([x, y - dist, radians(90)])
init_poses.append([x, y - 2*dist, radians(90)])
init_poses.append([x + dist, y -2*dist, radians(90)])
init_poses.append([x + 2 * dist, y - 2*dist, radians(90)])
init_poses.append([x + 2 * dist, y - dist, radians(90)])
init_poses.append([x + 2 * dist, y , radians(90)])
init_poses.append([x + dist, y, radians(90)])
init_poses.append([x + dist, y-dist, radians(90)])
init_poses.append([2, 2, radians(90)])

hub_location = init_poses[-1]

faulty_robot = [] # 8 : ran out of battery, 3 : stopped because of 8

FORMATION_BOT_IDS = [0, 1, 2, 3, 4, 5, 6, 7, 8]

# OMPL parameter

Break1 = 50
Break2 = 100

bound_limit = 8.00
newPoseX = []
newPoseY = []

#stanley parameters
stan_klm = 5  # control gain
stan_Kp = 2.0  # speed propotional gain
stan_dt = dt  # [s] time difference
stan_L = 0.23  # [m] Wheel base of vehicle
stan_max_steer = radians(50.0)  # [rad] max steering angle4
stan_target_speed = .25  # [m/s]