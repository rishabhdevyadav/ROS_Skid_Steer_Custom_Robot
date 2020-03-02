#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
import RPi.GPIO as GPIO

class wheelEncodersLeft():
    def __init__(self):
        rospy.init_node('wheelencoders_left', anonymous=True)
        self.left_enc_pub = rospy.Publisher('lwheel_counts', Int64, queue_size=10)
       
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)

        self.PinEncoderLeft = 17
        self.leftCount = 0

        GPIO.setup(self.PinEncoderLeft,GPIO.IN)
        GPIO.add_event_detect(self.PinEncoderLeft, GPIO.RISING, callback=self.leftShaft)
    
    def leftShaft(self, channel):
        self.leftCount += 1
        
    def spin(self):
        rospy.loginfo("Reading Left Encoder Counts")
        rate = rospy.Rate(200)
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self.left_enc_pub.publish(self.leftCount)
            rate.sleep()
        rospy.spin()

    def shutdown(self):
        self.leftCount = 0
        rospy.loginfo("Stopped Reading Left Encoder Counts")
        self.left_enc_pub.publish(0)
        rospy.sleep(1)

def main():
    wheel_Encoders = wheelEncodersLeft();
    wheel_Encoders.spin()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
