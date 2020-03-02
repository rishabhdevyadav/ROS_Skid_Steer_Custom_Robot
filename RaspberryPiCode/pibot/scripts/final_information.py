#!/usr/bin/python
from __future__ import division
# coding: utf-8

import rospy
import tf
from numpy import radians, degrees
from std_msgs.msg import Float64 
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""def callback(msg):
    theta = msg.vector.z
    if theta > radians(180):
        theta = radians(360) - theta
    if theta < -radians(180):
        theta = radians(360) + theta
    theta = -theta
    mag_pub.publish(degrees(theta))"""

def callback1(msg):  #wheel_odom
    encd_pos_x = msg.pose.pose.position.x
    encd_pos_y = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    end_ori_yaw = yaw
    #print('wheel_odom : ',encd_pos_x,encd_pos_y,degrees(end_ori_yaw))

def callback2(msg):  #d_odom
    deca_pos_x = msg.pose.pose.position.x
    deca_pos_y = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    imu_yaw_world = yaw
    #print('d_odom : ',deca_pos_x,deca_pos_y,degrees(imu_yaw_world))

def callback3(msg):  #f_odom
    kalman_x = msg.pose.pose.position.x
    kalman_y = msg.pose.pose.position.y

    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    kalman_yaw = yaw
    #print('f_odom : ',kalman_x,kalman_y,degrees(kalman_yaw))


try:
    rospy.init_node('pose_data', anonymous=True)
    # rospy.on_shutdown(shutdown)
    rate = rospy.Rate(7)
    mag_pub = rospy.Publisher('imu/yaw_deg', Float64, queue_size=10)

    robot_id = rospy.get_namespace().replace('/', '')

    #rospy.loginfo("HEADING IN DEG")
    #rospy.Subscriber('imu/rpy/filtered', Vector3Stamped, callback) 
    rospy.Subscriber('/{}/wheel_odom'.format(robot_id), Odometry, callback1)
    rospy.Subscriber('/{}/d_odom'.format(robot_id), Odometry, callback2)
    rospy.Subscriber('/{}/f_odom'.format(robot_id), Odometry, callback3) 

    while True and not rospy.is_shutdown():
        try:
            rospy.spin()
        except Exception as e:
            rospy.logerr(e)

except Exception as e:
    rospy.logerr(e)
