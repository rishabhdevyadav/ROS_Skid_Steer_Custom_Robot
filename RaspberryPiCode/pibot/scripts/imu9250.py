#!/usr/bin/python
from __future__ import division
# coding: utf-8

import rospy
import tf
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

import MPU9250
from math import radians

MAG_HARD_BIAS  =  (113.90899999999999, -40.54, -16.3515)
MAG_SOFT_BIAS = (0.9602207761635727, 0.9829804630346844, 1.0624072704609615)

#MAG_HARD_BIAS  =  (132.1605, -30.133499999999998, -23.2225)
#MAG_SOFT_BIAS = (1.0291878517105106, 0.9204656212387662, 1.061623641331525)

G_TO_MS2 = 9.8

mpu9250 = MPU9250.MPU9250()

# shutdown ROS on interrupt
def shutdown():
    rospy.loginfo("Shutting Down Ploting")
    rospy.sleep(1)

try:
    rospy.init_node('IMU_Plotting', anonymous=True)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(7)
    imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)

    imu_msg = Imu()
    mag_msg = MagneticField()
    rospy.loginfo("IMU STARTED")

    while True and not rospy.is_shutdown():
        try:
            m9a = mpu9250.readAccel()
            m9g = mpu9250.readGyro()
            mag = mpu9250.readMagnet()

            m9a = [G_TO_MS2 * x for x in m9a]
            m9g = [radians(x) for x in m9g]

            mx, my, mz = ((mag[x] - MAG_HARD_BIAS[x]) * MAG_SOFT_BIAS[x] for x in range(3))

            # Fill mag msg
            mag_msg.header.stamp = rospy.get_rostime()
            mag_msg.magnetic_field.x = mx
            mag_msg.magnetic_field.y = my
            mag_msg.magnetic_field.z = mz

            # create imu msg
            q0 = 1.0 #W
            q1 = 0.0 #X
            q2 = 0.0 #Y
            q3 = 0.0 #Z

            #Fill imu message
            imu_msg.header.stamp = rospy.get_rostime()
            imu_msg.header.frame_id = 'imu_raw'

            imu_msg.orientation.x = q1
            imu_msg.orientation.y = q2
            imu_msg.orientation.z = q3
            imu_msg.orientation.w = q0
            imu_msg.orientation_covariance[0] = 1e6
            imu_msg.orientation_covariance[0] = 1e6
            imu_msg.orientation_covariance[0] = 0.1   

            imu_msg.angular_velocity.x = m9g[0]
            imu_msg.angular_velocity.y = m9g[1]
            imu_msg.angular_velocity.z = m9g[2]
            imu_msg.angular_velocity_covariance[0] = 1e6
            imu_msg.angular_velocity_covariance[4] = 1e6
            imu_msg.angular_velocity_covariance[8] = 0.1
            
            imu_msg.linear_acceleration.x = m9a[0]
            imu_msg.linear_acceleration.y = m9a[1]
            imu_msg.linear_acceleration.z = m9a[2]
            imu_msg.linear_acceleration_covariance[0] = 1e6
            imu_msg.linear_acceleration_covariance[4] = 1e6
            imu_msg.linear_acceleration_covariance[8] = 0.1

            imu_pub.publish(imu_msg)
            mag_pub.publish(mag_msg)

            rate.sleep()
        except KeyboardInterrupt:
            break
except rospy.ROSInterruptException:
    rospy.logwarn("ROS_NODE_ENDED")
except Exception as e:
    rospy.logerr('IMU NODE EXCEPTION: ', e)
