#!/usr/bin/env python
import os
import time
import rospy
import serial
from serial.serialutil import SerialException
from std_msgs.msg import Float32MultiArray

class DECAWAVE():
	def __init__(self):
		# initialize the decawave node 
		rospy.init_node('decawave', log_level=rospy.INFO)
		self.port = rospy.get_param("~port", "/dev/ttyS0")
		self.baud = int(rospy.get_param("~baud", 115200))
		self.timeout = rospy.get_param("~timeout", 0.5)
		# Overall loop rate: should be faster than fastest sensor rate
		self.rate = int(rospy.get_param("~rate", 10))

		# Publisher for all the distances
		self.distPub = rospy.Publisher('distances', Float32MultiArray, queue_size=10)
		i = 0
		while(i < 50):	
			try:
				self.ser = serial.Serial(self.port, self.baud)
				break		
			except SerialException as e:
				i = i+1 
				rospy.logerr(e)
				rospy.loginfo("Serial Port not Found")
				time.sleep(1)
				rospy.loginfo("Trying to Connect ... : %s", str(i))
		rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")


	def update(self):
		try:
			values = map(float, self.ser.readline().split())
			distData = Float32MultiArray()
			# print distData
			distData.data = values
			distData.layout.data_offset = len(values)
			self.distPub.publish(distData)
		except:
			print "distances recieved is not a float value"

	def spin(self):
		rospy.loginfo("Getting Distances from Decawave")
		rate = rospy.Rate(self.rate)
		# Cleanup when termniating the node
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update()
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Shutting Down Decawave Node")
		# Close the serial port
		try:
			self.ser.close()
		except:
			pass
		finally:
			rospy.loginfo("Serial port closed.")
			os._exit(0)

def main():
	decawave = DECAWAVE();
	decawave.spin()

if __name__ == '__main__':
	main()
