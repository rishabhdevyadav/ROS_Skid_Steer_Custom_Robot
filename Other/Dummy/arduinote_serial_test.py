#!/usr/bin/env python
import serial
import time
ser = serial.Serial('/dev/ttyS0', 115200)
try:
	while True:
		#ser.write('d\r')
		print ser.readline()
		#print ser.read(1)
		# time.sleep(0.5)
except:
	print("Error in execution")
finally:
	print("code exited successfully")
