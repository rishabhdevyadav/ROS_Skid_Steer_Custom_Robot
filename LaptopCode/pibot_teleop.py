#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Turtlebot!
---------------------------
Moving around:
        w     
   a    x    d
        s     

w     : increase linear speed by 0.05 m/sec
s     : decrease linear speed by -0.05 m/sec
a     : increase angular speed by 0.01 rad/sec
d     : decrease angular speed by -0.01 rad/sec
x     : stop the robot
space : stop the robot

CTRL-C to quit
"""

speedBindings={
        'w':(.05,0),
        'a':(0,.01),
        'd':(0,-.01),
        's':(-.05,0),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 0
turn = 0

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('pibot_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    max_speed = 0.30
    max_turn = 3.14
    try:
        print(msg)
        # print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in speedBindings.keys():
                speed = speed + speedBindings[key][0]
                turn = turn + speedBindings[key][1]

            elif key == ' ' or key == 'x' :
                speed = 0
                turn = 0

            elif (key == '\x03'):
                print("Closing Program")
                break

            else:
                pass

            if speed >= max_speed:
                speed = max_speed
            if speed <= -max_speed:
                speed = -max_speed
            if turn >= max_turn:
                turn = max_turn
            if turn <= -max_turn:
                turn = -max_turn

            # print(vels(speed,turn))
            twist = Twist()
            twist.linear.x = speed; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


