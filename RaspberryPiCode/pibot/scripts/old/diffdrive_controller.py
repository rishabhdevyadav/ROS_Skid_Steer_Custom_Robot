#!/usr/bin/python
import rospy
import roslib

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class CmdVelToDiffDriveMotors:
  def __init__(self):
    rospy.init_node('diffdrive_controller')
    self.cmdvel_sub = rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
    self.lwheel_angular_vel_target_pub = rospy.Publisher('lwheel_angular_vel_target', Float64, queue_size=10)
    self.rwheel_angular_vel_target_pub = rospy.Publisher('rwheel_angular_vel_target', Float64, queue_size=10)

    self.L = rospy.get_param('~robot_wheel_separation_distance', 0.23) #in mtr
    self.R = rospy.get_param('~robot_wheel_radius', 0.034) # in mtr

    self.rate = rospy.get_param('~rate', 10)
    self.timeout_idle = rospy.get_param('~timeout_idle', 2)
    self.time_prev_update = rospy.Time.now()

    self.target_v = 0;
    self.target_w = 0;

  def spin(self):
    rospy.loginfo("Start diffdrive_controller")
    rate = rospy.Rate(self.rate)
    time_curr_update = rospy.Time.now()
    rospy.on_shutdown(self.shutdown)
    while not rospy.is_shutdown():
      time_diff_update = (time_curr_update - self.time_prev_update).to_sec()
      if time_diff_update < self.timeout_idle:
        self.update();
      rate.sleep()
    rospy.spin();

  def shutdown(self):
    rospy.loginfo("Stop diffdrive_controller")
    self.lwheel_angular_vel_target_pub.publish(0)
    self.rwheel_angular_vel_target_pub.publish(0)
    rospy.sleep(1)   

    # vr = (2v + wL) / 2R
    # vl = (2v - wL) / 2R
  def update(self):
    vr = (2*self.target_v + self.target_w*self.L) / (2 * self.R)
    vl = (2*self.target_v - self.target_w*self.L) / (2 * self.R)
    self.rwheel_angular_vel_target_pub.publish(vr)
    self.lwheel_angular_vel_target_pub.publish(vl)

  def twistCallback(self,msg):
    self.target_v = msg.linear.x;
    self.target_w = msg.angular.z;
    self.time_prev_update = rospy.Time.now()


def main():
  cmdvel_to_motors = CmdVelToDiffDriveMotors();
  cmdvel_to_motors.spin()

if __name__ == '__main__':
  main(); 
