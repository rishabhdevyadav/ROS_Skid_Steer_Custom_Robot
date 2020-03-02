#!/usr/bin/python
import rospy
import roslib
import math 
import numpy
import tf

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist

class VisOdomPublisher:
  def __init__(self):
    rospy.init_node('Visual_Odometry')

    self.odom_pub = rospy.Publisher('viso', Odometry, queue_size=10)
    self.tf_listener = tf.TransformListener()

    self.rate = rospy.get_param('~rate', 10)
    self.frame_id = rospy.get_param('~frame_id','odom')
    self.child_frame_id = rospy.get_param('~child_frame_id','base_link')

    self.time_start = rospy.Time.now()
    rospy.sleep(2)
    self.tf_listener.waitForTransform('/world', '/camera_position', rospy.Time(), rospy.Duration(2.0))
    self.initial_pose_at_origin = 0
    self.x_init_err = 0
    self.y_init_err = 0
    self.theta_init_err = 0

  def update(self):
    (trans,rot) = self.tf_listener.lookupTransform('/world', '/camera_position', rospy.Time(0))
    quaternion = (rot[0], rot[1], rot[2], rot[3])
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)

    new_x = trans[2] - self.x_init_err
    new_y = trans[1] - self.y_init_err
    new_th = pitch - self.theta_init_err
    
    if self.initial_pose_at_origin == 0:
      self.initial_pose_at_origin = 1
      self.x_init_err = trans[2]
      self.y_init_err = trans[1]
      self.theta_init_err = pitch

    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = self.frame_id
    odom_msg.child_frame_id = self.child_frame_id
    odom_msg.pose.pose.position.x = -new_x #ZY-->XY   (Z = 2, Y = 1, X = 0) -- pitch  = our theta
    odom_msg.pose.pose.position.y = new_y
    odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,new_th))
    self.odom_pub.publish(odom_msg)       


  def spin(self):
    rospy.loginfo("Start Visual Odometry")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)
    while not rospy.is_shutdown():
      self.update();
      rate.sleep()
    rospy.spin()

  def shutdown(self):
    rospy.loginfo("Stop Visual Odometry")
    rospy.sleep(1)

def main():
  visodom_publisher = VisOdomPublisher();
  visodom_publisher.spin()

if __name__ == '__main__':
  main(); 
