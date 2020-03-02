#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
import RPi.GPIO as GPIO

class wheelEncoders():
	def __init__(self):

		rospy.init_node('wheelencoders', anonymous=True)
		self.left_enc_pub  = rospy.Publisher('lwheel_counts', Int64, queue_size=10)
		self.right_enc_pub = rospy.Publisher('rwheel_counts', Int64, queue_size=10)
		
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)

		self.PinEncoderLeft = 17
		self.PinEncoderRight = 4

		self.rightCount = 0
		self.leftCount = 0

		self.time_start = rospy.Time.now()

		GPIO.setup(self.PinEncoderRight,GPIO.IN)
		GPIO.setup(self.PinEncoderLeft,GPIO.IN)

		GPIO.add_event_detect(self.PinEncoderLeft, GPIO.RISING, callback=self.leftShaft)
		GPIO.add_event_detect(self.PinEncoderRight, GPIO.RISING, callback=self.rightShaft)
	
	def leftShaft(self, channel):
		self.leftCount += 1
		

	def rightShaft(self, channel):
		self.rightCount += 1
		

	def spin(self):
		rospy.loginfo("Reading Encoder Counts")
		# rate = rospy.Rate(10)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			# pass	
			time_duration = (rospy.Time.now() - self.time_start).to_sec()
			if time_duration > .1:
				time_duration = 0
				self.left_enc_pub.publish(self.leftCount)
				self.right_enc_pub.publish(self.rightCount)
				self.time_start = rospy.Time.now()
		# rospy.spin()

	def shutdown(self):
		rospy.loginfo("Stopped Reading Encoder Counts")
		rospy.sleep(1)

def main():
	wheel_Encoders = wheelEncoders();
	wheel_Encoders.spin()
	GPIO.cleanup()

if __name__ == '__main__':
	main()

