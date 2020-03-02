#!/usr/bin/env python
# license removed for brevity
import rospy
# from rocon_std_msgs.msg import StringArray
from custom_msgs.msg import botData
from std_msgs.msg import String
import time

class talker():

    def __init__(self):
        self.fback_list=[]
        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        time.sleep(1)

    def callback(self,data):
        print(data.data)
        self.fback_list.append(data.data)
        self.fback_list.sort()
        return


    def talk(self):
        pub = rospy.Publisher('chatter', botData, queue_size=10)

        a = botData()
        while not rospy.is_shutdown():
            a.botIds = ["3", "9"]
            a.botType = ["Leader", "Follower"]
            a.botFile = ["", "follower.launch"]
            rospy.loginfo(a)
            pub.publish(a)
            break
            self.rate.sleep()
            
        fback_list.append("3")
        rospy.Subscriber('fback',String,self.callback)
        ids_list=a.botIds
        ids_list.sort()
        while(ids_list!=self.fback_list):
            g=1
            
        self.fback_list=[]
        a.botIds = ["3", "9"]
        a.botType = ["Leader", "Follower"]
        a.botFile = ["leader_launcher.launch", ""]
        rospy.loginfo(a)
        pub.publish(a)

        print("*****************Successful************")

        rospy.spin()


def main():
    t = talker()
    t.talk()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

import rospy
