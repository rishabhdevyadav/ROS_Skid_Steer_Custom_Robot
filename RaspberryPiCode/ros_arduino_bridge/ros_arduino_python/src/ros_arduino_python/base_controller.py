#!/usr/bin/env python

"""
    A base controller class for the Arduino microcontroller
    
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses

KINEMATIC MODEL
         ^^
 ___________________      
|        ||         |
|BM1     ||      AM1|
|        ||         |
|BM2     ||      AM2|
|________||_________|

R = RADIUS OF WHEEL   &&   B = WHEEL TRACK 

V = R/2 * [AM1/2 + BM1/2 + AM2/2 + BM2/2]
W = R/2 * [AM1/B - BM1/B + AM2/B - BM2/B]

AM1 = 1/R * [V + B/2]
BM1 = 1/R * [V - B/2]
AM2 = 1/R * [V + B/2]
BM2 = 1/R * [V - B/2]

USED FOR ENCODER & SPEED & DISTANCE MEASUREMENT

LEFT WHEEL = (BM1 + BM2)/2
RIGHT WHEEL = (AM1 + AM2)/2

"""
from __future__ import division
import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
import os
import tf
import numpy as np
from math import sin, cos, pi
from std_msgs.msg import Float64, Int8, Int8MultiArray
from geometry_msgs.msg import Quaternion, Twist, Pose
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from math import radians, degrees, sqrt
from tf.transformations import quaternion_from_euler

""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, arduino, base_frame, name="base_controllers"):
        self.arduino = arduino
        self.name = name
        self.base_frame = base_frame
        self.robot_id = rospy.get_namespace().replace('/', '')
        self.frame_id = rospy.get_param("~frame_id", "odom_wheel")
        self.frame_id = '{}_{}'.format(self.robot_id, self.frame_id)
        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
        self.stopped = False
                 
        pid_params = dict()
        pid_params['wheel_diameter'] = rospy.get_param("~wheel_diameter", "") 
        pid_params['wheel_track'] = rospy.get_param("~wheel_track", "")
        #pid_params['wheel_track_WE'] = rospy.get_param("~wheel_track_WE", "")
        pid_params['encoder_resolution'] = rospy.get_param("~encoder_resolution", "") 

        pid_params['gear_reduction'] = rospy.get_param("~gear_reduction", 1.0)
        pid_params['Kp'] = rospy.get_param("~Kp", 40)
        pid_params['Kd'] = rospy.get_param("~Kd", 10)
        pid_params['Ki'] = rospy.get_param("~Ki", 0)
        pid_params['Ko'] = rospy.get_param("~Ko", 50)
        
        self.accel_limit = rospy.get_param('~accel_limit', 0.1)
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
        
        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)
            
        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * pi)
        
        # What is the maximum acceleration we will tolerate when changing
        # wheel speeds?
        self.max_accel = (self.accel_limit * self.ticks_per_meter) / self.rate
                
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0
                        
        now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta
        self.last_batt_update_time = rospy.Time.now()

        # Internal data        
        self.encoders = np.zeros(4)
        self.time = 0.0 #used for debug

#------------------------------------------
        # Internal data        
        self.enc_left = 0            # encoder readings
        self.enc_right = 0
        self.x = 0#0.28284271        # position in xy plane
        self.y = 0
        self.th = 0                  # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0           # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = now
#--------------------------------------------

                           # rotation in radians
        self.w = {
            'AM1': 0, 'BM1': 0, 'AM2':0, 'BM2': 0
        }
        self.target_w = {
            'AM1': 0, 'BM1': 0, 'AM2':0, 'BM2': 0
        }
        self.last_cmd_vel = now

        # voltage and Current Publisher
        self.VoltPub = rospy.Publisher('voltage', Float64, queue_size=5)
        self.CurrPub = rospy.Publisher('current', Float64, queue_size=5)
        # self.headingPub = rospy.Publisher('heading', Float64, queue_size=5)

        self.batt_flag = 0

        # Subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        rospy.Subscriber('pistonListing', Int8MultiArray, self.pistonCB)
 
        self.myID = 3

        self.pistonSubs = []#Int8MultiArray()
        
        # Clear any old odometry info
        self.arduino.reset_encoders()
        
        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('wheel_odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
        
        """rospy.loginfo(
            "Started base controller for a base of {}" +
            " x {} m^2 with {} ticks/rev" 
            .format(
                self.wheel_track_NS, self.wheel_track_WE,
                self.encoder_resolution
            )
        )"""
        rospy.loginfo(
            "Publishing odometry data at: {} Hz using {} as base frame".format(
                self.rate, self.base_frame
            ) 
        )

        r = self.wheel_diameter/2 #r=wheel_radius
        b = self.wheel_track  #METERS     #wheel_tarck  b=distance_between_wheels

        self.J_fwd = np.array([
            [1/2, 1/2, 1/2, 1/2],
            [1/b, -1/b, 1/b, -1/b]
        ])*(r/2)        

        self.J_inv = np.array([
            [1, b/2],
            [1, -b/2],
            [1, b/2],
            [1, -b/2],
        ])*(1/r)

        self.ticks_per_PID_rate = (self.encoder_resolution/(self.arduino.PID_RATE*2*np.pi))

        self.dist_test=0


    def pistonCB(self, msg):
        # 1 - UP
        # 2 - DOWN
        # else - STOP
        self.pistonSubs = msg.data

    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                print("*** PID Parameter {} is missing. ***".format(param))
                missing_params = True
        if missing_params:
            os._exit(1)
        self.__dict__.update(pid_params)        
        self.arduino.update_pid(self.Kp, self.Kd, self.Ki, self.Ko)

    def poll(self):

        now = rospy.Time.now()
        if (now - self.last_batt_update_time).to_sec() > 2:
            self.batt_flag = 1
        else:
            self.batt_flag = 0
        if now > self.t_next:
            try:
                if (self.myID in self.pistonSubs[1:]):
                    self.arduino.controlPiston(self.pistonSubs[0])
                if self.batt_flag == 1:
                    self.last_batt_update_time = rospy.Time.now()
                    self.VoltPub.publish(self.arduino.get_voltage()*0.9917)
                    self.CurrPub.publish(self.arduino.get_current())
                    self.batt_flag = 0
                encoder_counts = self.arduino.get_encoder_counts()
            except Exception as e :
                self.bad_encoder_count += 1
                return

            encoder_counts_AM1 = -encoder_counts[0]
            encoder_counts_BM1 = -encoder_counts[1]
            encoder_counts_AM2 = -encoder_counts[2]
            encoder_counts_BM2 = -encoder_counts[3]
            #print(encoder_counts)
            left_enc = (encoder_counts_BM1 + encoder_counts_BM2)/2
            right_enc = (encoder_counts_AM1 + encoder_counts_AM2)/2

            dt = now - self.then
            self.then = now
            dt = dt.to_sec()

            current_w = -1*((encoder_counts - self.encoders)*2*np.pi/(self.encoder_resolution)/dt)
            #print(current_w)

            #fwd = np.matmul(self.J_fwd, current_w).flatten()

            #self.encoders = np.array(encoder_counts)
            
            # Calculate odometry
            if self.enc_left == 0:
                dright = 0
                dleft = 0
            else:
                dright = (right_enc - self.enc_right) / self.ticks_per_meter
                dleft = (left_enc - self.enc_left) / self.ticks_per_meter

            self.enc_right = right_enc
            self.enc_left = left_enc

            #print('left:  ',dleft)
            #print('right: ',dright)

            #print('self.wheel_track', self.wheel_track)
            
            dxy_ave = ((dright + dleft) / 2.0)
            dth = ((dright - dleft) / self.wheel_track)
            vxy = (dxy_ave / dt)
            vth = (dth / dt)

            self.time += dt
            #print('dt : ',self.time)

            #print('v :',vxy)
            #print('w :',vth)

            if (dxy_ave != 0):
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                #print("dx : ",dx)
                #print("dy : ",dy)
                self.x += (cos(self.th) * dx - sin(self.th) * dy)
                self.y += (sin(self.th) * dx + cos(self.th) * dy)
    
            if (dth != 0):
                self.th += dth 


            #q = quaternion_from_euler(0, 0, self.th)
            quaternion = Quaternion(*tf.transformations.quaternion_from_euler(0,0,self.th))

            #quaternion = Quaternion()
            #quaternion.x = q[0] 
            #quaternion.y = q[1]
            #quaternion.z = q[2]
            #quaternion.w = q[3]

            #print(degrees(self.th*7.11))
    
            # Create the odometry transform frame broadcaster.
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0), 
                (quaternion.x,quaternion.y,quaternion.z,quaternion.w),
                rospy.Time.now(),
                self.base_frame,
                self.frame_id
                )
    
            odom = Odometry()
            odom.header.frame_id = self.frame_id
            odom.child_frame_id = self.base_frame
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.pose.covariance = [0.5,     0,    0,    0,    0,    0, \
                                       0,  0.5,    0,    0,    0,    0, \
                                       0,     0,  1e2,    0,    0,    0, \
                                       0,     0,    0,  1e6,    0,    0, \
                                       0,     0,    0,    0,  1e6,    0, \
                                       0,     0,    0,    0,    0,  1e3]
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = vth

            self.odomPub.publish(odom)
            
            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                self.target_w = {key: 0 for key in self.target_w.keys()}

            for key in self.target_w.keys():
                diff = self.w[key] - self.target_w[key]
                if diff < 0:
                    self.w[key] += min(self.max_accel, abs(diff))
                if diff > 0:
                    self.w[key] -= min(self.max_accel, abs(diff))
            if not self.stopped:
                self.arduino.drive(**self.w)

            self.t_next = now + self.t_delta
                
    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0)


    def cmdVelCallback(self, req):
        self.last_cmd_vel = rospy.Time.now()
        v_lin = req.linear.x         # m/s
        w_ang = req.angular.z       # rad/s

        target = np.matmul(self.J_inv, [[v_lin],[w_ang]]).flatten()
        

        (self.target_w['AM1'], self.target_w['BM1'], self.target_w['AM2'],self.target_w['BM2']) = \
        -1*(target*self.ticks_per_PID_rate).astype('i')
