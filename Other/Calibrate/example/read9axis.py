#!/usr/bin/python
from __future__ import division
# coding: utf-8

import MPU9250
import datetime
import sys
import time
import fusion 
from math import pi
# from matplotlib import pyplot as plt

import rospy
import tf
from std_msgs.msg import Float64

MAG_CALIBRATE = True

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
        print "Time elapsed : {} seconds".format(delta.seconds)
        return 0


if MAG_CALIBRATE:
    fuse.calibrate(getmag, stopfunc, 200) #wait for 200 ms between consecutive values
    # print "Bias : {}".format(fuse.maghardbias)
    sys.exit(1)

try:
    rospy.init_node('IMU_Plotting', anonymous=True)
    rospy.on_shutdown(shutdown)
    heading_pub = rospy.Publisher('heading', Float64, queue_size=10)
    # fig = plt.gcf()
    # fig.show()
    # fig.canvas.draw()
    count = 0
    while True and not rospy.is_shutdown():
        count = count + 1
        accel = mpu9250.readAccel()
        gyro = mpu9250.readGyro()
        mag = mpu9250.readMagnet()
        time.sleep(0.2)
        gyro = [x * pi/180.0 for x in gyro]

        delta = datetime.datetime.now() - ts
        ts = datetime.datetime.now()
        fuse.update(accel, gyro, mag, delta.microseconds/1000000.0)
        fuse.heading =  (fuse.heading + 360) % 360;
        # heading_pub.publish(fuse.heading)
        print fuse.heading

        # plt.plot(fuse.heading,count) # plot something
        # fig.canvas.draw() 

        # print "------------------------------", fuse.heading 
        # print "accel : ", accel
        # print "gyro : ", gyro
        # print "mag : ", mag
        # print "heading : {}".format(fuse.heading)#, delta.microseconds/1000000.0)

except KeyboardInterrupt:
    sys.exit()
