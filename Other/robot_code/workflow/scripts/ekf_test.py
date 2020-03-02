#!/usr/bin/env python
# Author : Abhishek Raj Dutta
# Date :22/11/2106
# This code merges stereo odometry with IMU and command velocities to produce an estimate of the HUSKY's pose and heading


import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math
import numpy as np

pub=rospy.Publisher('/odometry/filtered/self',Odometry,queue_size=10)
br = tf.TransformBroadcaster()
odom=Odometry()
imu=Imu()
yawViso=0.0
vYawViso=0.0
vX=0.0
yawImu=0.0
yawImu1=0.0
yaw=0.0
i=0
vYawCov=0.09
ImuYawCov=0.15707963267948966
yawCov=10
cmd = np.array ([(0.0,0.0),(0.0,0.0)])
yaw = np.array ([0.0,0.0])
meas = np.array ([0.0,0.0])
inp = np.array ([0.0,0.0])
odomEKF = np.array ([0.0,0.0,0.0])
pred = np.array ([0.0,0.0,0.0])
prevVx=0.0
#frequency = 5 Hz
f=50.0
t=1/f


def viso_callback(msg):    
	global odom
	odom=msg
	quaternion1 = (
	msg.pose.pose.orientation.x,
	msg.pose.pose.orientation.y,
	msg.pose.pose.orientation.z,
	msg.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion1)
	roll = euler[0]
	pitch = euler[1]
	global yawViso,vYawViso,vX
	yawViso = euler[2]
	vYawViso=msg.twist.twist.angular.z;
	vX=msg.twist.twist.linear.x;

	# if vX > 1 :
	#     vX = prevVx
	# else :
	#     prevVx = vX
		
	
	

def imu_callback(msg):
	global imu
	imu=msg
	quaternion = (
	msg.orientation.x,
	msg.orientation.y,
	msg.orientation.z,
	msg.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	global yawImu
	yawImu = euler[2]
	

def getCmd(msg):
	global cmd
	cmd[1][0] = msg.linear.x
	cmd[1][1] = msg.angular.z

def send():
	odomOut=Odometry()
	odomOut.header.stamp=rospy.Time.now()
	odomOut.header.frame_id = "odom";
	odomOut.child_frame_id = "base_link";
	odomOut.pose.pose.position.x=odomEKF[0]
	odomOut.pose.pose.position.y=odomEKF[1]
	quaternion = tf.transformations.quaternion_from_euler(0, 0, odomEKF[2])
	odomOut.pose.pose.orientation.x = quaternion[0]
	odomOut.pose.pose.orientation.y = quaternion[1]
	odomOut.pose.pose.orientation.z = quaternion[2]
	odomOut.pose.pose.orientation.w = quaternion[3]
	
	br.sendTransform((odomEKF[0], odomEKF[1], 0),
					 quaternion,
					 rospy.Time.now(),
					 "base_link",
					 "odom")

	
	#type(pose) = geometry_msgs.msg.Pose
	

	pub.publish(odomOut)
	# print odomEKF


def loop(event):
	print 'Timer called at ' + str(event.current_real)
	global i,yawImu,yawImu1,vYawViso,vYawCov,ImuYawCov,yaw,yawCov,vX,odomEKF,pred,cmd, inp, prevVx
	if i==0:
		yawImu1=yawImu
		i=1
	
	#linear KF
	predYaw=yaw[0]+vYawViso*t
	predCov=yawCov + vYawCov
	inn=(yawImu-yawImu1)-predYaw
	innCov=predCov+ImuYawCov
	K=predCov*(1/innCov)
	yaw[1]=predYaw+K*inn
	yawCov=(1-K)*predCov
	vYaw=(yaw[1]-yaw[0])*f
	yaw[0]=yaw[1]


	meas[1]=vYaw
	if vX<1:
		meas[0]=vX
		prevVx = vX
	else: 
		meas[0]=prevVx

	vConCov=0.0001
	wConCov=0.001

	vMeasCov=0.002
	wMeasCov=yawCov*f

	inp[0]=(((vConCov)**2)*meas[0]+((vMeasCov)**2)*cmd[0][0])/(((vConCov)**2)+((vMeasCov)**2))
	inp[1]=(((wConCov)**2)*meas[1]+((wMeasCov)**2)*cmd[0][1])/(((wConCov)**2)+((wMeasCov)**2))

	if cmd[0][0]>0.000001 or cmd[0][1]>0.000001:
		
		if cmd[0][1]>0.000001:
			pred[0]=-(inp[0]/inp[1])*np.sin(odomEKF[2])+(inp[0]/inp[1])*np.sin(odomEKF[2]+inp[1]*t)
			pred[1]=(inp[0]/inp[1])*np.cos(odomEKF[2])-(inp[0]/inp[1])*np.cos(odomEKF[2]+inp[1]*t)
			pred[2]=inp[1]*t

		else:
			pred[0]=inp[0]*t*np.cos(yaw[0])
			pred[1]=inp[0]*t*np.sin(yaw[1])
			#pred[1]=(inp[0]/inp[1])*np.cos(odomEKF[2])-(inp[0]/inp[1])*np.cos(odomEKF[2]+inp[1]*t)
			#pred[1]=(meas[0]/meas[1])*np.cos(odomEKF[2])-(meas[0]/meas[1])*np.cos(odomEKF[2]+meas[1]*t)
			pred[2]=meas[1]*t

		odomEKF=odomEKF+pred

	

	cmd[0][0]=cmd[1][0]
	cmd[0][1]=cmd[1][1]
	rospy.loginfo(cmd)
	#print odomEKF   
	send()

def ekf():
	rospy.init_node('ekf_node')
	rospy.Subscriber("odom", Odometry, viso_callback)
	# rospy.Subscriber("/husky_velocity_controller/odom", Odometry, viso_callback)
	rospy.Subscriber("cmd_vel", Twist, getCmd)
	rospy.Subscriber("imu", Imu, imu_callback)
	rospy.Timer(rospy.Duration(t), loop)
	rospy.spin()

if __name__ == '__main__':
	ekf()