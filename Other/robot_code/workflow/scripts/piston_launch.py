import rospy
import time
import subprocess

def Piston():
	pkg_name = 'workflow'
	file_name = 'piston_launcher.launch'
	r=[1,2,3]
	s=2
	child = subprocess.Popen(["roslaunch", pkg_name, file_name, "robot_list1:={}".format(r),"piston_status1:={}".format(s)])


if __name__ == '__main__':
	Piston()