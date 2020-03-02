#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
import RPi.GPIO as GPIO

class wheelEncodersRight():
    def __init__(self):

        rospy.init_node('wheelencoders_right', anonymous=True)
        self.right_enc_pub = rospy.Publisher('rwheel_counts', Int64, queue_size=10)
        
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.PinEncoderRight = 26
        self.rightCount = 0

        GPIO.setup(self.PinEncoderRight,GPIO.IN)
        GPIO.add_event_detect(self.PinEncoderRight, GPIO.RISING, callback=self.rightShaft)
    
    def rightShaft(self, channel):
        self.rightCount += 1

    def spin(self):
        rospy.loginfo("Reading Right Encoder Counts")
        rate = rospy.Rate(200)
        rospy.on_shutdown(self.shutdown)
        while not rospy.is_shutdown():
            self.right_enc_pub.publish(self.rightCount)
            rate.sleep()
        rospy.spin()
        
    def shutdown(self):
        self.rightCount = 0
        rospy.loginfo("Stopped Reading Right Encoder Counts")
        self.right_enc_pub.publish(0)
        rospy.sleep(1)

def main():
    wheel_Encoders = wheelEncodersRight();
    wheel_Encoders.spin()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
