#!/usr/bin/env python
import rospy
import time
from math import sin, cos, pi, radians
from nav_msgs.msg import Odometry
import tf

class GenerateMAP:
	def __init__(self):
		rospy.init_node('generate_map', anonymous=True)
		self.rate = rospy.get_param('~rate', 20)
		# initializations
		self.robot_poses = []
		# number of robots in arena
		self.bot_ids = [3, 5, 6, 7, 8, 9]

		# ID of the robot to be replaced
		self.newRobotID = 6
		# ID of the leader robot wrt to the failed robot
		self.LeaderRobotID = 8	
		# Robot to be replaced must calculate the goal position wrt leader
		self.distance = 0.5 # in meters
		self.angle = -90 * pi / 180 # in degrees

		# Global pose of all the robots present in the arena
		for i in self.bot_ids:
			self.cbPose(rospy.wait_for_message('/R'+str(i)+ '/f_odom', Odometry))

	def cbPose(self, msg):
		temp = []
		temp.append(msg.pose.pose.position.x)
		temp.append(msg.pose.pose.position.y)
		orientation = msg.pose.pose.orientation
		quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		temp.append(euler[2])
		self.robot_poses.append(temp)	

	# find the starting point of the new robot to be replaced with the discharged robot
	def getStartPoint(self):
		start_point = ""
		for i, j in enumerate(self.bot_ids):
			if j == self.newRobotID:
				start_point = self.robot_poses[i]
				start_point = (" ".join(str(x) for x in start_point))
				start_point = str(j) + " " + start_point
				break
			else:
				# print("Error in selecting the New Robot, Please recheck the ID specified for new robot")
				start_point = "0 0 0 0"
		return start_point

	# find the position of the leader to the failed robot
	def getLeaderPosition(self):
		leader_pose = []
		for i, j in enumerate(self.bot_ids):
			if j == self.LeaderRobotID:
				leader_pose = self.robot_poses[i]
				break
			else:
				leader_pose = []
				pass
		return leader_pose

	# calculate the goal position of the robot to be replaced with the discharged robot
	def getGoalPoint(self):
		goal = []
		leader_pose = self.getLeaderPosition()
		goal.append(leader_pose[0] + self.distance * cos(leader_pose[2] + self.angle))
		goal.append(leader_pose[1] + self.distance * sin(leader_pose[2] + self.angle))
		goal.append(leader_pose[2] + radians(30))
		goal = (" ".join(str(x) for x in goal))
		return str(self.newRobotID) + " " + goal

	# find the positions of the other robots acting as obstacles in the arena
	def getObstacles(self):
		obstacles = ""
		obs = ""
		for i, j in enumerate(self.bot_ids):
			if not j == self.newRobotID:
				obs = self.robot_poses[i]
				obs = (" ".join(str(x) for x in obs))
				obs = str(j) + " " + obs
				obstacles = obstacles + obs + "\n"
			else:
				pass
		return obstacles

	# create a string of all the points
	def createStringData(self):
		time.sleep(2)
		start = self.getStartPoint()
		goal = self.getGoalPoint()
		obstacles = self.getObstacles()
		string = obstacles + start + "\n" + goal
		print string
		return string

	# Write all the poses in a file
	def createPosesFile(self, data):
		file = open("/home/pulkit/Documents/Pose.txt", "w")
		file.write(data)

	# Run the loop to create the pose file
	def spin(self):
		rospy.loginfo("Finding Obstacle Positions")
		self.createPosesFile(self.createStringData())
		rospy.loginfo("Shutting down...Obstacle Positions found")

# Main loop
def main():
	gmap = GenerateMAP();
	gmap.spin()

# call main
if __name__ == '__main__':
	main(); 