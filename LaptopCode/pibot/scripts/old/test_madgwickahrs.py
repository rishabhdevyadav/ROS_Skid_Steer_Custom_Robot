#!/usr/bin/env python
from __future__ import division

import rospy
import tf
from numpy import degrees, pi
from numpy import linalg as LA

class Madgwick:
	def __init__(self):
		self.beta = 0.1
		self.q0 = 1.0
		self.q1 = 0.0
		self.q2 = 0.0
		self.q3 = 0.0
		self.invSampleFreq = 1.0 / 512
		self.angle = 0

	def update(self, gyro, accel):
		gx, gy, gz = gyro
		ax, ay, az = accel

		# Rate of change of quaternion from gyroscope
		qDot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
		qDot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
		qDot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
		qDot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

		if ax!=0 and ay!=0 and az!=0 :
			recipNorm = LA.norm([ax, ay, az])
			ax /= recipNorm
			ay /= recipNorm
			az /= recipNorm

			# Auxiliary variables to avoid repeated arithmetic
			_2q0 = 2. * self.q0
			_2q1 = 2. * self.q1
			_2q2 = 2. * self.q2
			_2q3 = 2. * self.q3
			_4q0 = 4. * self.q0
			_4q1 = 4. * self.q1
			_4q2 = 4. * self.q2
			_8q1 = 8. * self.q1
			_8q2 = 8. * self.q2
			q0q0 = self.q0 * self.q0
			q1q1 = self.q1 * self.q1
			q2q2 = self.q2 * self.q2
			q3q3 = self.q3 * self.q3

			# Gradient decent algorithm corrective step
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * self.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
			s2 = 4.0 * q0q0 * self.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
			s3 = 4.0 * q1q1 * self.q3 - _2q1 * ax + 4.0 * q2q2 * self.q3 - _2q2 * ay
			recipNorm = LA.norm([s0, s1, s2, s3])# normalise step magnitude
			s0 /= recipNorm;
			s1 /= recipNorm;
			s2 /= recipNorm;
			s3 /= recipNorm;

			# Apply feedback step
			qDot1 -= self.beta * s0;
			qDot2 -= self.beta * s1;
			qDot3 -= self.beta * s2;
			qDot4 -= self.beta * s3;
		
		#Integrate rate of change of quaternion to yield quaternion
		self.q0 += qDot1 * self.invSampleFreq;
		self.q1 += qDot2 * self.invSampleFreq;
		self.q2 += qDot3 * self.invSampleFreq;
		self.q3 += qDot4 * self.invSampleFreq;
		
		recipNorm = LA.norm([self.q0, self.q1, self.q2, self.q3])
		self.q0 /= recipNorm;
		self.q1 /= recipNorm;
		self.q2 /= recipNorm;
		self.q3 /= recipNorm;
		self.angle = degrees(tf.transformations.euler_from_quaternion((self.q0, self.q1, self.q2, self.q3))[2])



	def spin(self):
		rospy.loginfo("Madgwick filter starting")
		rate = rospy.Rate(10)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update()
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Shutting Down Madgwick filter")
		rospy.sleep(1)

def main():
	mad = Madgwick();
	mad.spin()

if __name__ == '__main__':
	main()
