#!/usr/bin/python
import rospy
import roslib
import smbus
import time
import math

from std_msgs.msg import Float64

class IMU:
	def __init__(self):
		rospy.init_node('imu_init')
		self.imu_pub = rospy.Publisher('imu', Float64, queue_size=10)

		self.rate = rospy.get_param('~rate', 500)
		self.time_prev_update = rospy.Time.now()

		self.bus = smbus.SMBus(1)
		self.address = 0x1e
		self.write_byte(0, 0b01110000) # Set to 8 samples @ 15Hz
		self.write_byte(1, 0b00100000) # 1.3 gain LSb / Gauss 1090 (default)
		self.write_byte(2, 0b00000000) # Continuous sampling
		self.scale = 0.92

	def read_byte(self, adr):
		return self.bus.read_byte_data(self.address, adr)

	def read_word(self, adr):
		high = self.bus.read_byte_data(self.address, adr)
		low = self.bus.read_byte_data(self.address, adr+1)
		val = (high << 8) + low
		return val

	def read_word_2c(self, adr):
		val = self.read_word(adr)
		if (val >= 0x8000):
			return -((65535 - val) + 1)
		else:
			return val

	def write_byte(self, adr, value):
		self.bus.write_byte_data(self.address, adr, value)


	def spin(self):
		rospy.loginfo("Start imu")
		rate = rospy.Rate(self.rate)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update();
			rate.sleep()
		rospy.spin();

	def shutdown(self):
		rospy.loginfo("Stop imu")
		self.imu_pub.publish(0.0)
		rospy.sleep(1)   

	def update(self):
		x_out = self.read_word_2c(3) * self.scale
		y_out = self.read_word_2c(7) * self.scale
		z_out = self.read_word_2c(5) * self.scale
		bearing  = math.atan2(y_out, x_out)
		if (bearing < 0):
			bearing += 2 * math.pi
		val = math.degrees(bearing)
		self.imu_pub.publish(val)

def main():
  imu = IMU();
  imu.spin()

if __name__ == '__main__':
  main(); 
