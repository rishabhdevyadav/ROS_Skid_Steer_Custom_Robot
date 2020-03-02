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
files_to_run =[]
count=0
blocked_ip = []
def runfile(file):
    global child
    # child = subprocess.Popen(["rosrun","beginner_tutorials",file])
    child = subprocess.Popen(["roslaunch","pibot",file])

def get_ip():
    cmd="hostname -I"
    ip=subprocess.check_output(cmd,shell=True)
    ip= ip.split(" ")
    ip=ip[0]
    ip=ip.rstrip()
    return ip



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
    print("got data")
    global files_to_run
    files_to_run.append(data.data)
    global count
    print("count..........................",count)

    if(count==0):
        count=1
        run_files()
    return

    # global file
    # file=data.data
    # runfile(data.data)
    # print("Done.........................")

def callback_stop(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
    if data.data==file or data.data=="all":
        child.terminate()

def callback_blocked_ip(data):
    rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
    
    global blocked_ip
    blocked_ip.append(data.data)



def run_files():
    flag=1;
    ip=get_ip()
    pool=cycle(files_to_run)
    for item in pool:
        for ips in blocked_ip:
            if(ip==ips):
                flag=0;
                print("IP Blocked..........")
                break;
        if flag==1:
            runfile(item)
            time.sleep(30)
            child.terminate()
        if flag==0:
            print("IP Blocked....")
            return


def listener():

    rospy.init_node('listener1', anonymous=True)

    rospy.Subscriber('chatter', String, callback)
    rospy.Subscriber('stop',String,callback_stop)
    rospy.Subscriber('blocked',String,callback_blocked_ip)

    rospy.spin()

if __name__ == '__main__':
    listener()


# export ROS_MASTER_URI=http://10.2.129.143:11311
# export ROS_IP=10.2.132.101
