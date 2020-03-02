#!/usr/bin/python
from __future__ import division
import rospy
import roslib
import math 
import numpy
from std_msgs.msg import Float64, Int64


class ControlsToMotors:
  def __init__(self):
    rospy.init_node('pibot_controller')
    self.rate = rospy.get_param('~rate', 1000)
    #self.Kp = rospy.get_param('~Kp', 3.5)
    #self.Ki = rospy.get_param('~Ki', 2.2)
    #self.Kd = rospy.get_param('~Kd', 0.0)

    self.R = rospy.get_param('~robot_wheel_radius', 0.034) #in mtr
    self.Total_Counts = rospy.get_param('~Counts per rotation', 1072) 
    self.pid_on = rospy.get_param('~pid_on',True)

    self.time_prev_update = rospy.Time.now();

    self.lwheel_angular_vel_enc_pub = rospy.Publisher('lwheel_angular_vel_enc', Float64, queue_size=10)
    self.rwheel_angular_vel_enc_pub = rospy.Publisher('rwheel_angular_vel_enc', Float64, queue_size=10)


    self.lwheel_error_pub = rospy.Publisher('lwheel_error_motor', Float64, queue_size=10)
    self.rwheel_error_pub = rospy.Publisher('rwheel_error_motor', Float64, queue_size=10)

    self.lwheel_sub = rospy.Subscriber('lwheel_counts', Int64, self.lwheel_callback)
    self.rwheel_sub = rospy.Subscriber('rwheel_counts', Int64, self.rwheel_callback)

    self.lwheel_angular_vel_target_sub = rospy.Subscriber('lwheel_angular_vel_target', Float64, self.lwheel_angular_vel_target_callback)
    self.rwheel_angular_vel_target_sub = rospy.Subscriber('rwheel_angular_vel_target', Float64, self.rwheel_angular_vel_target_callback)

    self.lwheel_angular_vel_target = 0;
    self.rwheel_angular_vel_target = 0;

    self.lwheel_cps_target = 0
    self.rwheel_cps_target = 0

    self.lwheel_angular_vel_enc = 0;
    self.rwheel_angular_vel_enc = 0;

    self.lwheel_cps_actual = 0
    self.rwheel_cps_actual = 0

    self.lwheel_encs_counts = [0]*2
    self.rwheel_encs_counts = [0]*2

    self.lwheel_counts = 0
    self.rwheel_counts = 0

    self.lwheel_pid = {}
    self.rwheel_pid = {}



  def lwheel_angular_vel_target_callback(self, msg):
    self.lwheel_angular_vel_target = msg.data

  def rwheel_angular_vel_target_callback(self, msg):
    self.rwheel_angular_vel_target = msg.data

  def lwheel_callback(self,msg):
    self.lwheel_counts = msg.data

  def rwheel_callback(self,msg):
    self.rwheel_counts = msg.data
 
  def calc_cps(self, angular_vel):
    const = ( (2.0 * math.pi) / self.Total_Counts )
    counts = angular_vel * const
    return counts

  def calc_angular_vel(self, cps):
    const = ( self.Total_Counts / (2.0 * math.pi) )
    ang_vel = cps * const
    return ang_vel

  def pid_control(self,wheel_pid,target,state):
    if len(wheel_pid) == 0:
      wheel_pid.update({'time_prev':rospy.Time.now(), 'derivative':0, 'integral':[0]*10, 'error_prev':0,'error_curr':0})

    wheel_pid['time_curr'] = rospy.Time.now()

    wheel_pid['dt'] = (wheel_pid['time_curr'] - wheel_pid['time_prev']).to_sec()
    if wheel_pid['dt'] == 0: return 0

    wheel_pid['error_curr'] = target - state

    wheel_pid['error_curr']  = wheel_pid['error_curr'] / 67

    wheel_pid['integral'] = wheel_pid['integral'][1:] + [(wheel_pid['error_curr']*wheel_pid['dt'])]
    wheel_pid['derivative'] = (wheel_pid['error_curr'] - wheel_pid['error_prev'])/wheel_pid['dt']

    wheel_pid['error_prev'] = wheel_pid['error_curr']
    control_signal = (self.Kp*wheel_pid['error_curr'] + self.Ki*sum(wheel_pid['integral']) + self.Kd*wheel_pid['derivative'])

    wheel_pid['time_prev'] = wheel_pid['time_curr']
    return control_signal

  def update_left_counts(self):
    self.lwheel_encs_counts = self.lwheel_encs_counts[1:] + [self.lwheel_counts] #slicing

    time_curr_update = rospy.Time.now()
    dt = (time_curr_update - self.time_prev_update).to_sec()

    cps = self.lwheel_encs_counts[-1] - self.lwheel_encs_counts[-2]
    
    self.time_prev_update = time_curr_update
    return cps

  def update_right_counts(self):
    self.rwheel_encs_counts = self.rwheel_encs_counts[1:] + [self.rwheel_counts] #slicing

    time_curr_update = rospy.Time.now()
    dt = (time_curr_update - self.time_prev_update).to_sec()

    cps = self.rwheel_encs_counts[-1] - self.rwheel_encs_counts[-2]
    
    self.time_prev_update = time_curr_update
    return cps


  def lwheel_update(self):
    lwheel_cps_target = self.calc_cps(self.lwheel_angular_vel_target)
    lwheel_cps_actual = self.update_left_counts()

    self.lwheel_angular_vel_enc = self.calc_angular_vel(lwheel_cps_actual)
    self.lwheel_angular_vel_enc_pub.publish(self.lwheel_angular_vel_enc)

    #self.Kp = 2;self.Ki = 10;self.Kd = 0
    self.Kp = 10
    self.Ki = 0
    self.Kd = 0
    if self.pid_on: 
      err_val = self.pid_control(self.lwheel_pid, lwheel_cps_target, lwheel_cps_actual)

    self.lwheel_error_pub.publish(err_val)    

  def rwheel_update(self):
    rwheel_cps_target = self.calc_cps(self.rwheel_angular_vel_target)
    rwheel_cps_actual = self.update_right_counts()
    
    self.rwheel_angular_vel_enc = self.calc_angular_vel(rwheel_cps_actual)
    self.rwheel_angular_vel_enc_pub.publish(self.rwheel_angular_vel_enc)

    #self.Kp = 1;self.Ki = 0;self.Kd = 0.5
    self.Kp = 10
    self.Ki = 0
    self.Kd = 0

    if self.pid_on: 
      err_val= self.pid_control(self.rwheel_pid, rwheel_cps_target, rwheel_cps_actual)

    self.rwheel_error_pub.publish(err_val)    

  def spin(self):
    rospy.loginfo("Start pibot_controller")
    rate = rospy.Rate(self.rate)
    
    rospy.on_shutdown(self.shutdown)

    while not rospy.is_shutdown():
      self.rwheel_update()
      self.lwheel_update()
      rate.sleep()
    rospy.spin();

  def shutdown(self):
    rospy.loginfo("Stop pibot_controller")
    self.lwheel_error_pub.publish(0)
    self.rwheel_error_pub.publish(0)
    rospy.sleep(1)        

def main():
  controls_to_motors = ControlsToMotors();
  controls_to_motors.spin()

if __name__ == '__main__':
  main(); 

