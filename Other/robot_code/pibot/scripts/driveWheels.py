#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool
import RPi.GPIO as GPIO

class driveWheels():
	def __init__(self):

		rospy.init_node('moveMotors')
		
		self.left_pwm_pub  = rospy.Publisher('l_pwm', Float64, queue_size=10)
		self.right_pwm_pub = rospy.Publisher('r_pwm', Float64, queue_size=10)

		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.BCM)

		self.PinMotorRight1 =5
		self.PinMotorRight2 = 6
		self.RightMotorEnable = 13

		self.PinMotorLeft1 = 27
		self.PinMotorLeft2 = 22
		self.LeftMotorEnable = 18

		GPIO.setup(self.RightMotorEnable,GPIO.OUT)
		GPIO.setup(self.PinMotorRight1,GPIO.OUT)
		GPIO.setup(self.PinMotorRight2,GPIO.OUT)

		GPIO.setup(self.LeftMotorEnable,GPIO.OUT)
		GPIO.setup(self.PinMotorLeft1,GPIO.OUT)
		GPIO.setup(self.PinMotorLeft2,GPIO.OUT)

		self.pwmLeftWheel = 0.0
		self.pwmRightWheel = 0.0

		self.enL = GPIO.PWM(self.LeftMotorEnable,1000)
		self.enR = GPIO.PWM(self.RightMotorEnable,1000)

		rospy.Subscriber('lwheel_error_motor', Float64, self.callback_left)
		rospy.Subscriber('rwheel_error_motor', Float64, self.callback_right)
		rospy.Subscriber('goalReached', Bool, self.callback_stop)


	def forward(self):
		GPIO.output(self.PinMotorLeft1,GPIO.HIGH)
		GPIO.output(self.PinMotorLeft2,GPIO.LOW)
		GPIO.output(self.PinMotorRight1,GPIO.HIGH)
		GPIO.output(self.PinMotorRight2,GPIO.LOW)

	def backward(self):
		GPIO.output(self.PinMotorLeft1,GPIO.LOW)
		GPIO.output(self.PinMotorLeft2,GPIO.HIGH)
		GPIO.output(self.PinMotorRight1,GPIO.LOW)
		GPIO.output(self.PinMotorRight2,GPIO.HIGH)

	def pwmConfig(self, str, val):
		str.start(50)
		str.ChangeDutyCycle(val)

	def pwmStop(self):
		self.enL.stop()
		self.enR.stop()

	def callback_stop(self, msg):
		if msg.data: 
			self.pwmLeftWheel = 0
			self.pwmRightWheel = 0

	def callback_left(self, msg):
		self.pwmLeftWheel = msg.data + self.pwmLeftWheel
		if self.pwmLeftWheel > 100:
	  		self.pwmLeftWheel = 100
		if self.pwmLeftWheel < 0:
	  		self.pwmLeftWheel = 0
	  	self.left_pwm_pub.publish(self.pwmLeftWheel)
		self.pwmConfig(self.enL, self.pwmLeftWheel)

	def callback_right(self, msg):
		self.pwmRightWheel = msg.data + self.pwmRightWheel
		if self.pwmRightWheel > 100:
	  		self.pwmRightWheel = 100
		if self.pwmRightWheel < 0:
	  		self.pwmRightWheel = 0
	  	self.right_pwm_pub.publish(self.pwmRightWheel)
		self.pwmConfig(self.enR, self.pwmRightWheel)

	def spin(self):
		rospy.loginfo("Start driveWheels")
		rate = rospy.Rate(50)
		rospy.on_shutdown(self.shutdown)
		self.forward()
		while not rospy.is_shutdown():
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		self.pwmRightWheel = 0
		self.pwmLeftWheel = 0
		rospy.loginfo("Stop driveWheels")
		self.pwmStop()
		self.left_pwm_pub.publish(0)
		self.right_pwm_pub.publish(0)
		rospy.sleep(1)

def main():
	drive_Wheels = driveWheels();
	drive_Wheels.spin()
	GPIO.cleanup()


if __name__ == '__main__':
	main()
