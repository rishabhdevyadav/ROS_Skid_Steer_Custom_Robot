#!/usr/bin/python
from __future__ import division

import rospy
import json
from math import sin, cos
import roslib
import smbus
import time

import tf
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion, Vector3


class GYRO:

	# Global Variables
	GRAVITIY_MS2 = 9.80665

	# Scale Modifiers
	ACCEL_SCALE_MODIFIER_2G = 16384.0
	ACCEL_SCALE_MODIFIER_4G = 8192.0
	ACCEL_SCALE_MODIFIER_8G = 4096.0
	ACCEL_SCALE_MODIFIER_16G = 2048.0

	GYRO_SCALE_MODIFIER_250DEG = 131.0
	GYRO_SCALE_MODIFIER_500DEG = 65.5
	GYRO_SCALE_MODIFIER_1000DEG = 32.8
	GYRO_SCALE_MODIFIER_2000DEG = 16.4

	# Pre-defined ranges
	ACCEL_RANGE_2G = 0x00
	ACCEL_RANGE_4G = 0x08
	ACCEL_RANGE_8G = 0x10
	ACCEL_RANGE_16G = 0x18

	GYRO_RANGE_250DEG = 0x00
	GYRO_RANGE_500DEG = 0x08
	GYRO_RANGE_1000DEG = 0x10
	GYRO_RANGE_2000DEG = 0x18

	# MPU-6050 Registers
	PWR_MGMT_1 = 0x6B
	PWR_MGMT_2 = 0x6C

	ACCEL_XOUT0 = 0x3B
	ACCEL_YOUT0 = 0x3D
	ACCEL_ZOUT0 = 0x3F

	TEMP_OUT0 = 0x41

	GYRO_XOUT0 = 0x43
	GYRO_YOUT0 = 0x45
	GYRO_ZOUT0 = 0x47

	ACCEL_CONFIG = 0x1C
	GYRO_CONFIG = 0x1B

	def __init__(self):
		rospy.init_node('imu_init')
		self.imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
		self.rate = rospy.get_param('~rate', 10)
		self.dT = 1 / self.rate
		self.time_prev_update = rospy.Time.now()

		self.frame_id = rospy.get_param('~frame_id','imu')

		self.bus = smbus.SMBus(1)
		self.address = 0x68
		self.theta = 0
		self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

	def read_i2c_word(self, register):
		"""Read two i2c registers and combine them.

		register -- the first register to read from.
		Returns the combined read results.
		"""
		# Read the data from the registers
		high = self.bus.read_byte_data(self.address, register)
		low = self.bus.read_byte_data(self.address, register + 1)

		value = (high << 8) + low

		if (value >= 0x8000):
			return -((65535 - value) + 1)
		else:
			return value

	# MPU-6050 Methods

	def get_temp(self):
		"""Reads the temperature from the onboard temperature sensor of the MPU-6050.

		Returns the temperature in degrees Celcius.
		"""
		raw_temp = self.read_i2c_word(self.TEMP_OUT0)

		# Get the actual temperature using the formule given in the
		# MPU-6050 Register Map and Descriptions revision 4.2, page 30
		actual_temp = (raw_temp / 340.0) + 36.53

		return actual_temp

	def set_accel_range(self, accel_range):
		"""Sets the range of the accelerometer to range.

		accel_range -- the range to set the accelerometer to. Using a
		pre-defined range is advised.
		"""
		# First change it to 0x00 to make sure we write the correct value later
		self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

		# Write the new range to the ACCEL_CONFIG register
		self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

	def read_accel_range(self, raw = False):
		"""Reads the range the accelerometer is set to.

		If raw is True, it will return the raw value from the ACCEL_CONFIG
		register
		If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
		returns -1 something went wrong.
		"""
		raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

		if raw is True:
			return raw_data
		elif raw is False:
			if raw_data == self.ACCEL_RANGE_2G:
				return 2
			elif raw_data == self.ACCEL_RANGE_4G:
				return 4
			elif raw_data == self.ACCEL_RANGE_8G:
				return 8
			elif raw_data == self.ACCEL_RANGE_16G:
				return 16
			else:
				return -1

	def get_accel_data(self, g = False):
		"""Gets and returns the X, Y and Z values from the accelerometer.

		If g is True, it will return the data in g
		If g is False, it will return the data in m/s^2
		Returns a dictionary with the measurement results.
		"""
		x = self.read_i2c_word(self.ACCEL_XOUT0)
		y = self.read_i2c_word(self.ACCEL_YOUT0)
		z = self.read_i2c_word(self.ACCEL_ZOUT0)

		accel_scale_modifier = None
		accel_range = self.read_accel_range(True)

		if accel_range == self.ACCEL_RANGE_2G:
			accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
		elif accel_range == self.ACCEL_RANGE_4G:
			accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
		elif accel_range == self.ACCEL_RANGE_8G:
			accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
		elif accel_range == self.ACCEL_RANGE_16G:
			accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
		else:
			print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
			accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

		x = x / accel_scale_modifier
		y = y / accel_scale_modifier
		z = z / accel_scale_modifier

		if g is True:
			return {'x': x, 'y': y, 'z': z}
		elif g is False:
			x = x * self.GRAVITIY_MS2
			y = y * self.GRAVITIY_MS2
			z = z * self.GRAVITIY_MS2
			return {'x': x, 'y': y, 'z': z}

	def set_gyro_range(self, gyro_range):
		"""Sets the range of the gyroscope to range.

		gyro_range -- the range to set the gyroscope to. Using a pre-defined
		range is advised.
		"""
		# First change it to 0x00 to make sure we write the correct value later
		self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

		# Write the new range to the ACCEL_CONFIG register
		self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

	def read_gyro_range(self, raw = False):
		"""Reads the range the gyroscope is set to.

		If raw is True, it will return the raw value from the GYRO_CONFIG
		register.
		If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
		returned value is equal to -1 something went wrong.
		"""
		raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

		if raw is True:
			return raw_data
		elif raw is False:
			if raw_data == self.GYRO_RANGE_250DEG:
				return 250
			elif raw_data == self.GYRO_RANGE_500DEG:
				return 500
			elif raw_data == self.GYRO_RANGE_1000DEG:
				return 1000
			elif raw_data == self.GYRO_RANGE_2000DEG:
				return 2000
			else:
				return -1

	def get_gyro_data(self):
		"""Gets and returns the X, Y and Z values from the gyroscope.

		Returns the read values in a dictionary.
		"""
		x = self.read_i2c_word(self.GYRO_XOUT0)
		y = self.read_i2c_word(self.GYRO_YOUT0)
		z = self.read_i2c_word(self.GYRO_ZOUT0)

		gyro_scale_modifier = None
		gyro_range = self.read_gyro_range(True)

		if gyro_range == self.GYRO_RANGE_250DEG:
			gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
		elif gyro_range == self.GYRO_RANGE_500DEG:
			gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
		elif gyro_range == self.GYRO_RANGE_1000DEG:
			gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
		elif gyro_range == self.GYRO_RANGE_2000DEG:
			gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
		else:
			print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
			gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

		x = x / gyro_scale_modifier
		y = y / gyro_scale_modifier
		z = z / gyro_scale_modifier

		return {'x': x, 'y': y, 'z': z}

	def get_all_data(self):
		"""Reads and returns all the available data."""
		temp = self.get_temp()
		accel = self.get_accel_data()
		gyro = self.get_gyro_data()
		return [accel, gyro, temp]

	def spin(self):
		rospy.loginfo("Start gyro")
		rate = rospy.Rate(self.rate)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update();
			rate.sleep()
		rospy.spin();

	def shutdown(self):
		rospy.loginfo("Stop gyro")
		rospy.sleep(1)  


# d = {"angular_velocity": {"x": av.x, "y": av.y, "z": av.z},
#        "linear_acceleration": {"x": lv.x, "y": lv.y, "z": lv.z}}
# print json.dumps(d) 

	def update(self):
		
		[linear_accel, angular_vel, temp] = self.get_all_data()

		imu_msg = Imu()
		imu_msg.header.stamp = rospy.Time.now()
		imu_msg.header.frame_id = self.frame_id
		imu_msg.angular_velocity.x = angular_vel['x']
		imu_msg.angular_velocity.y = angular_vel['y']
		imu_msg.angular_velocity.z = angular_vel['z']
		imu_msg.linear_acceleration.x = linear_accel['x']
		imu_msg.linear_acceleration.y = linear_accel['y']
		imu_msg.linear_acceleration.z = linear_accel['z']
		self.imu_pub.publish(imu_msg)
		# gyro_data = self.get_gyro_data()
		# val = gyro_data['z']
		# -0.09209 || -0.1005 offset compensation of Gyro (will vary if change in rate occurs)
		# self.theta = -0.0997 + self.theta + val * self.dT
		# if self.theta > 360 or self.theta < -360:
			# self.theta = 0
		# self.imu_pub1.publish(val) #for debug
		# self.imu_angles_pub.publish(self.theta) # for debug
		# self.pub_imu(val)
		#self.imu_pub.publish(self.theta)

	# def pub_imu(self,val):
	# 	imu_msg = Imu()
	# 	imu_msg.header.stamp = rospy.Time.now()
	# 	imu_msg.header.frame_id = self.frame_id
	# 	imu_msg.orientation.w = cos(self.theta/2)#Quaternion(*tf.transformations.quaternion_from_euler(0,0,self.theta))
	# 	imu_msg.orientation.z = sin(self.theta/2)
	# 	#imu_msg.angular_velocity = Vector3(0,0,val)
	# 	self.imu_pub.publish(imu_msg)

def main():
	gyro = GYRO();
	gyro.spin()


if __name__ == "__main__":
	main()
