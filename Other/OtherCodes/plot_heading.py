#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3Stamped
from matplotlib import pyplot as plt
from math import degrees


class GET_HEADING():
    def __init__(self):
        rospy.init_node('GET_HEADING')
        # rospy.Subscriber('/heading', Float64, self.cbHeading)   
        rospy.Subscriber('/R3/imu/rpy/filtered', Vector3Stamped, self.cbHeading)   
        # rospy.Subscriber('/R3/imu/rpy/filtered', Vector3Stamped, self.cbHeading5)   
        self.heading = 0.0
        self.heading5 = 0.0
        self.counter = 0
        self.rpy = Vector3Stamped()
        # self.rpy5 = Vector3Stamped()

    # def cbHeading(self, msg):
        # self.heading = msg.data
        # self.heading = (self.heading + 360) % 360;

    def cbHeading(self, msg):
        # print msg.vector
        self.rpy = msg.vector
        self.heading = degrees(self.rpy.z)
        self.heading = (self.heading + 360) % 360
        print self.heading

    # def cbHeading5(self, msg):
    #     # print msg.vector
    #     self.rpy5 = msg.vector
    #     self.heading5 = degrees(self.rpy5.z)
    #     self.heading5 = (self.heading5 + 360) % 360
    #     print self.heading5

    def update(self, fig):
        self.counter = self.counter + 1
        plt.plot(self.counter, self.heading, 'ro') # plot something
        # plt.plot(self.counter, self.heading5, 'bo') # plot something
        fig.canvas.draw() 
        plt.pause(0.01)

    def spin(self):
        rospy.loginfo("Get heading")
        rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown)
        fig = plt.gcf()
        fig.show()
        fig.canvas.draw()
        while not rospy.is_shutdown():
            self.update(fig)
            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting Down Heading")
        rospy.sleep(1)

def main():
    GETHEADING = GET_HEADING();
    GETHEADING.spin()

if __name__ == '__main__':
    main()
