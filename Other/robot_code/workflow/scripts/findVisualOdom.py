#!/usr/bin/python
import rospy
import roslib
import math 
import numpy
import tf

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Pose2D

class OdomPublisher:
  def __init__(self):
    rospy.init_node('diffdrive_odom')
    self.ID = rospy.get_param('~ID', "000")
    self.pose_sub = rospy.Subscriber('/cosphi_ros_bridge/robot' + self.ID, Pose2D, self.pose_callback)    
    self.odom_pub = rospy.Publisher('odom1', Odometry, queue_size=10)

    self.rate = rospy.get_param('~rate', 10)
    self.frame_id = rospy.get_param('~frame_id','/odom1')
    self.child_frame_id = rospy.get_param('~child_frame_id','/base_link')

    self.tf_broadcaster = tf.TransformBroadcaster()

    self.pose = {'x':0, 'y': 0, 'th': 0}
    self.time_prev_update = rospy.Time.now();

    self.time_start = rospy.Time.now()

  def pose_callback(self, msg):
    self.pose = {'x':msg.x, 'y':msg.y, 'th':msg.theta}



  def pub_odometry(self,pose):
    
    odom_msg = Odometry()
    odom_msg.header.stamp = self.time_prev_update
    odom_msg.header.frame_id = self.frame_id
    odom_msg.child_frame_id = self.child_frame_id
    odom_msg.pose.pose.position = Point(pose['y'], pose['x'], 0)
    odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose['th']))
    self.odom_pub.publish(odom_msg)

  def pub_tf(self,pose):
    self.tf_broadcaster.sendTransform( \
                              (pose['y'], pose['x'], 0), \
                              tf.transformations.quaternion_from_euler(0,0,pose['th']), \
                              self.time_prev_update, \
                              self.child_frame_id, \
                              self.frame_id \
                              )

  def update(self):
    self.pose['th'] = -self.pose['th'] * (math.pi / 180) # in radians
    self.pub_odometry(self.pose)
    self.pub_tf(self.pose)
      
  def spin(self):
    rospy.loginfo("Start diffdrive_odom")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)
    while not rospy.is_shutdown():
      self.update();
      rate.sleep()
    rospy.spin()

  def shutdown(self):
    rospy.loginfo("Start diffdrive_odom")
    rospy.sleep(1)

def main():
  odom_publisher = OdomPublisher();
  odom_publisher.spin()

if __name__ == '__main__':
  main(); 

