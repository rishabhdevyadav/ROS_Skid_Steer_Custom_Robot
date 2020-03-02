#!/usr/bin/python
from __future__ import division
# coding: utf-8

import MPU9250
import datetime
import sys
import time
import fusion 
import math
import numpy as np

import rospy
import tf
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

MAG_CALIBRATE = False
ACC_CALIBRATE = False

MAG_HARD_BIAS  =  (32.905, -16.378999999999998, -18.526)#(7.462, 9.591999999999999, -40.0095)
MAG_SOFT_BIAS  =  (0.9704012441679626, 0.5989740124101492, 3.333589776467068)#(1.0676170789859354, 0.912653446647781, 1.0334545531540975)

G_TO_MS2 = 9.8

mpu9250 = MPU9250.MPU9250()
fuse = fusion.Fusion()
ts = datetime.datetime.now()

# shutdown ROS on interrupt
def shutdown():
    rospy.loginfo("Shutting Down Ploting")
    rospy.sleep(1)

def getmag():                  # Return (x, y, z) tuple (blocking read)
    val = mpu9250.readMagnet()
    print "mag : ", val
    return val 

def stopfunc():
    global ts
    delta = datetime.datetime.now() - ts
    if delta.seconds == 120:
        return 1
    else:
        return 0


if MAG_CALIBRATE:
    fuse.calibrate(getmag, stopfunc, 200) #wait for 200 ms between consecutive values
    sys.exit(1)

try:
    rospy.init_node('IMU_Plotting', anonymous=True)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(5)
    imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)

    imu_msg = Imu()
    mag_msg = MagneticField()
    rospy.loginfo("IMU STARTED")

    while True and not rospy.is_shutdown():
        m9a = mpu9250.readAccel()
        m9g = mpu9250.readGyro()
        mag = mpu9250.readMagnet()

        m9a = [G_TO_MS2 * x for x in m9a]
        m9g = [math.radians(x) for x in m9g]

        mx, my, mz = ((mag[x] - MAG_HARD_BIAS[x]) * MAG_SOFT_BIAS[x] for x in range(3))

        # Fill mag msg
        mag_msg.header.stamp = rospy.get_rostime()
        mag_msg.magnetic_field.x = mx#mag[0]
        mag_msg.magnetic_field.y = my#mag[1]
        mag_msg.magnetic_field.z = mz#mag[2]

        # create imu msg
        q0 = 1.0 #W
        q1 = 0.0 #X
        q2 = 0.0 #Y
        q3 = 0.0 #Z

        #Fill imu message
        imu_msg.header.stamp = rospy.get_rostime()

        imu_msg.orientation.x = q1
        imu_msg.orientation.y = q2
        imu_msg.orientation.z = q3
        imu_msg.orientation.w = q0
        imu_msg.orientation_covariance[0] = 0
        imu_msg.orientation_covariance[0] = 0
        imu_msg.orientation_covariance[0] = 0    

        imu_msg.angular_velocity.x = m9g[0]
        imu_msg.angular_velocity.y = m9g[1]
        imu_msg.angular_velocity.z = m9g[2]
        imu_msg.angular_velocity_covariance[0] = 0
        imu_msg.angular_velocity_covariance[4] = 0
        imu_msg.angular_velocity_covariance[8] = 0
        
        imu_msg.linear_acceleration.x = m9a[0]
        imu_msg.linear_acceleration.y = m9a[1]
        imu_msg.linear_acceleration.z = m9a[2]
        imu_msg.linear_acceleration_covariance[0] = 0
        imu_msg.linear_acceleration_covariance[4] = 0
        imu_msg.linear_acceleration_covariance[8] = 0

        imu_pub.publish(imu_msg)
        mag_pub.publish(mag_msg)

        rate.sleep()

except KeyboardInterrupt:
    sys.exit(1)
except rospy.ROSInterruptException:
    print "ROS_NODE_ENDED"
