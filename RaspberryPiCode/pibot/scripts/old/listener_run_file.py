#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from itertools import cycle
import os
import time
import subprocess
import signal
global child
global file
from std_msgs.msg import Int32MultiArray
files_to_run =[]
count=0
def runfile(file):
    global child
    # child = subprocess.Popen(["rosrun","beginner_tutorials",file])
    child = subprocess.Popen(["roslaunch","pibot",file])
    # child = subprocess.Popen(["python",file])


def callback(data):
    global file

    rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
    if 3 in data.data:
        # file=data.data
        runfile("leader_launcher.launch")
        print("Done.........................")
    return

def callback_stop(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
    if data.data==file or data.data=="all":
        child.terminate()



def run_files():
    pool=cycle(files_to_run)
    for item in pool:
        runfile(item)
        time.sleep(30)
        child.terminate()


def listener():

    rospy.init_node('listener1', anonymous=True)

    # rospy.Subscriber('chatter', String, callback)
    rospy.Subscriber('distances', Int32MultiArray, callback)
    rospy.Subscriber('stop',String,callback_stop)

    rospy.spin()

if __name__ == '__main__':
    listener()


# export ROS_MASTER_URI=http://10.2.129.143:11311
# export ROS_IP=10.2.132.101
