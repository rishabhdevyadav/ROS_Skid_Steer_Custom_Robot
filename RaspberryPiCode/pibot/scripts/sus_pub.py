#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from pibot.msg import *
#from pibot.msg import CamPose, CamPoseArray
from nav_msgs.msg import Odometry
import tf
from geometry_msgs.msg import Point, Quaternion, PoseWithCovarianceStamped
from tf.transformations import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from custom_msgs.msg import CamPose, CamPoseArray
from geometry_msgs.msg import Twist
import time

camID = 3

def callback(msg):
    x = {i.botID: [i.pose.x, i.pose.y, i.pose.theta] for i in msg.cp_list}
    #print(x[camID][0])  #x
    #print(x[camID][1])  #y
    #print((x[camID][2]))  #theta

    robot_id = rospy.get_namespace().replace('/', '')
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "world"   #frame_id == world
    odom_msg.child_frame_id = "{}_pose".format(robot_id)   #child_frame_id == r3_base_link_wheel
    odom_msg.pose.pose.position.x = x[camID][0]
    odom_msg.pose.pose.position.y = x[camID][1]

    odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,np.deg2rad(x[camID][2])))
    odom_msg.pose.covariance =      [ 1.0 ,     0,    0,    0,    0,    0, \
                                        0,    1.0,    0,    0,    0,    0, \
                                        0,     0,  1e6,    0,    0,    0, \
                                        0,     0,    0,  1e6,    0,    0, \
                                        0,     0,    0,    0,  1e6,    0, \
                                        0,     0,    0,    0,    0,  1e3]
    odom_pub.publish(odom_msg)
    #print "----------"
    #print "----------"

if __name__ == '__main__':
    robot_id = rospy.get_namespace().replace('/', '')
    rospy.init_node('{}_pub_sub'.format(robot_id) , anonymous=True)

    rospy.Subscriber('/Robot_poses', CamPoseArray, callback)
    rate = rospy.Rate(10)
    odom_pub = rospy.Publisher('/{}/f_odom'.format(robot_id), Odometry, queue_size = 10)
    rospy.spin()
