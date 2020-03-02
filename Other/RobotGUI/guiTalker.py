#!/usr/bin/env python
import rospy
from custom_msgs.msg import botData
from std_msgs.msg import String, Int8MultiArray
from geometry_msgs.msg import Twist
import time
import json
import numpy as np
import subprocess
import signal
import psutil

class talker():

    def __init__(self):
        self.fback_list=[]
        rospy.init_node('botString', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.botDataPub = rospy.Publisher('botData', botData, queue_size=10)
        self.file_read_timeout = 0.1
        self.startTime = time.time()
        self.piston_pub = rospy.Publisher('pistonListing', Int8MultiArray, queue_size=10)
        self.filepath = '/home/avar/Documents/RobotGUI/config'
        time.sleep(1)

    def check_behavior(self, path=None, file='/gen_config.json'):
        with open (path + file, 'r') as f:
            data = json.load(f)
        return data['code']

    def erase_behavior(self, path=None, file='/gen_config.json'):
        data={}
        data['code'] = "None"
        with open (path + file, 'w') as f:
            json.dump(data, f)

    def talk(self):
        a = botData()

        while (1):
            try:
                if (time.time() - self.startTime) > self.file_read_timeout:
                    self.startTime = time.time()
                    behavior = self.check_behavior(path=self.filepath)
                
                # For controlling the Piston of a particular robot
                if behavior == "piston":
                    with open (self.filepath + '/piston.json', 'r') as f:
                        data = json.load(f)
                    robot_list = [int(i) for i in data['ids']]
                    robot_list.insert(0,int(data['status']))
                    piston_list = Int8MultiArray()
                    piston_list.data = np.array(robot_list)
                    self.piston_pub.publish(piston_list)
                    self.erase_behavior(path=self.filepath)


                # For controlling the Speed of a particular robot
                if behavior == "cmd_vel":
                    with open (self.filepath + '/cmd_vel.json', 'r') as f:
                        data = json.load(f)
                    rid = data['id']
                    vel = float(data['vel'])
                    omega = float(data['omega'])
                    cmd_vel_pub = rospy.Publisher('/R'+ rid + '/cmd_vel', Twist, queue_size=10)
                    msg = Twist()
                    msg.linear.x = vel
                    msg.angular.z = omega
                    self.erase_behavior(path=self.filepath)
                    while(1):
                        cmd_vel_pub.publish(msg)
                        if (time.time() - self.startTime) > self.file_read_timeout:
                            self.startTime = time.time()
                            behavior = self.check_behavior(path=self.filepath)
                            if behavior != "None":
                                break
                        self.rate.sleep()

                # Sending a robot to a Goal point
                if behavior == "g2g":
                    child = subprocess.Popen(["python", "/home/avar/Documents/omplapp-1.4.0-Source/demos/SE2RigidBodyPlanning/tracker.py"])
                    child.wait()
                    while psutil.pid_exists(child.pid):
                        pass
                    print "out from wait*********************8"
                    self.erase_behavior(path=self.filepath)

            except KeyboardInterrupt:
                print "exitting"
                exit()



def main():
    t = talker()
    t.talk()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

