#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from custom_msgs.msg import botData

from itertools import cycle
import os
import time
import subprocess
import signal
import netifaces as ni
global child
count=0
global robotId
def runfile(file):
    global child
    # child = subprocess.Popen(["rosrun","beginner_tutorials",file])
    child = subprocess.Popen(["roslaunch","workflow",file])
    # child = subprocess.Popen(["python","/home/avar/catkin_ws/src/beginner_tutorials/scripts/"+file])


def callback(data):
    global robotId
    ids=data.botIds
    file=data.botFile
    print("Got the File to run")
    if robotId in ids and file[ids.index(robotId)]!="":
        ind=ids.index(robotId)
        runfile(file[ind])

    print("Done.........................")
    return

def callback_stop(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
    if data.data==file or data.data=="all":
        child.terminate()



def listener():

    global robotId
    ip = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
    robotId=ip[-1:]
    print(robotId)
    rospy.init_node('listener3', anonymous=True)

    rospy.Subscriber('botData', botData, callback)
    rospy.Subscriber('stop',String,callback_stop)

    rospy.spin()

if __name__ == '__main__':
    listener()


# export ROS_MASTER_URI=http://10.2.129.143:11311
# export ROS_IP=10.2.132.101
