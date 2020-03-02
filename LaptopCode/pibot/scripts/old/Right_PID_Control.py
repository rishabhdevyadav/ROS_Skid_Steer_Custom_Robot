#!/usr/bin/python
from __future__ import division
import rospy
import roslib
import math 
import numpy
from std_msgs.msg import Float64, Int64


class ControlsToRightMotor:
  def __init__(self):
    rospy.init_node('right_control')

    self.rate = rospy.get_param('~rate',10)
    self.R = rospy.get_param('~robot_wheel_radius', 0.034) #in mtr
    self.Total_Counts = rospy.get_param('~Counts per rotation', 1072) 
    self.pid_on = rospy.get_param('~pid_on',True)
    self.Kp = rospy.get_param('~Kp') 
    self.Ki = rospy.get_param('~Ki')
    self.Kd = rospy.get_param('~Kd')

    self.time_prev_update = rospy.Time.now()

    self.rwheel_angular_vel_enc_pub = rospy.Publisher('rwheel_angular_vel_enc', Float64, queue_size=10)
    self.rwheel_cps_enc_pub = rospy.Publisher('rwheel_enc_cps', Float64, queue_size=10)
    self.rwheel_cps_target_pub = rospy.Publisher('rwheel_target_cps', Float64, queue_size=10)
    self.rwheel_error_pub = rospy.Publisher('rwheel_error_motor', Float64, queue_size=10)
    self.rwheel_sub = rospy.Subscriber('rwheel_counts', Int64, self.rwheel_callback)
    self.rwheel_angular_vel_target_sub = rospy.Subscriber('rwheel_angular_vel_target', Float64, self.rwheel_angular_vel_target_callback)

    self.rwheel_angular_vel_target = 0;
    self.rwheel_cps_target = 0
    self.rwheel_angular_vel_enc = 0;
    self.rwheel_cps_actual = 0
    self.rwheel_encs_counts_prev = 0
    self.rwheel_counts = 0
    self.rwheel_pid = {}

  def rwheel_angular_vel_target_callback(self, msg):
    self.rwheel_angular_vel_target = msg.data

  def rwheel_callback(self,msg):
    self.rwheel_counts = msg.data

  def calc_cps(self, angular_vel):
    const = ( self.Total_Counts / (2.0 * math.pi) )
    counts = angular_vel * const
    return counts

  def calc_angular_vel(self, cps):
    const = ( (2.0 * math.pi) / self.Total_Counts )
    ang_vel = cps * const
    return ang_vel

  def pid_control(self,wheel_pid,target,state):
    if len(wheel_pid) == 0:
      wheel_pid.update({'time_prev':rospy.Time.now(), 'derivative':0, 'integral':[0]*10, 'error_prev':0,'error_curr':0})

    wheel_pid['time_curr'] = rospy.Time.now()

    wheel_pid['dt'] = (wheel_pid['time_curr'] - wheel_pid['time_prev']).to_sec()
    if wheel_pid['dt'] == 0: return 0

    wheel_pid['error_curr'] = target - state

    wheel_pid['error_curr']  = wheel_pid['error_curr'] 

    wheel_pid['integral'] = wheel_pid['integral'][1:] + [(wheel_pid['error_curr']*wheel_pid['dt'])]
    wheel_pid['derivative'] = (wheel_pid['error_curr'] - wheel_pid['error_prev'])/wheel_pid['dt']

    wheel_pid['error_prev'] = wheel_pid['error_curr']
    control_signal = (self.Kp*wheel_pid['error_curr'] + self.Ki*sum(wheel_pid['integral']) + self.Kd*wheel_pid['derivative'])

    wheel_pid['time_prev'] = wheel_pid['time_curr']
    return control_signal

  def update_right_counts(self):
    dt = (rospy.Time.now() - self.time_prev_update).to_sec()
    cps = (self.rwheel_counts - self.rwheel_encs_counts_prev ) / dt
    self.rwheel_encs_counts_prev = self.rwheel_counts    
    self.time_prev_update = rospy.Time.now()
    return cps

  def rwheel_update(self):
    rwheel_cps_target = self.calc_cps(self.rwheel_angular_vel_target)# 2000 at 0.4 mtr/sec
    rwheel_cps_actual = self.update_right_counts()
    self.rwheel_cps_enc_pub.publish(rwheel_cps_actual)
    self.rwheel_cps_target_pub.publish(rwheel_cps_target)
    self.rwheel_angular_vel_enc = self.calc_angular_vel(rwheel_cps_actual)
    self.rwheel_angular_vel_enc_pub.publish(self.rwheel_angular_vel_enc)

    #self.Kp = 2;self.Ki = 10;self.Kd = 0
    self.Kp = .005
    self.Ki = 0
    self.Kd = 0
    if self.pid_on: 
      err_val = self.pid_control(self.rwheel_pid, rwheel_cps_target, rwheel_cps_actual)
    self.rwheel_error_pub.publish(err_val)    

  def spin(self):
    rospy.loginfo("Start Right_control")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)
    while not rospy.is_shutdown():
      self.rwheel_update()
      rate.sleep()
    rospy.spin();

  def shutdown(self):
    rospy.loginfo("Stop Right_control")
    self.rwheel_error_pub.publish(0)
    rospy.sleep(1)        

def main():
  controls_to_motors = ControlsToRightMotor();
  controls_to_motors.spin()

if __name__ == '__main__':
  main(); 

