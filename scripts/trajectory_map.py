#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from project2.srv import TrajectorySrv, TrajectorySrvResponse
import os

class Trajectory:
	def __init__(self):
		self.cwd = os.getcwd()  #script path
		rospy.Subscriber("/map", OccupancyGrid, self.callback_map)
		rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_trajectory)
		rospy.Service('trajectory', TrajectorySrv, self.write_map)
		self.map_ready = False

	# saves map data into a matrix
	def callback_map(self, data):
		self.width = data.info.width
		self.height = data.info.height
		size = (self.width , self.height)
		self.image = np.zeros(size)

		i = 0
		for pixel in data.data:  #copy image into a matrix
			self.image[i % self.width][int(i / self.width)] = pixel
			i+=1

		self.image[self.image == -1] = 50   #gray
		self.image[self.image == 0] = 255   #white
		self.image[self.image == 100] = 0   #black

		#parameters for calculating exact pixel positions
		self.resolution = data.info.resolution
		self.scaling = abs(data.info.origin.position.x) - self.width * self.resolution / 2.0;

		self.map_ready = True

	# create trajectory of points
	def callback_trajectory(self, data):
		if(self.map_ready):
			x = data.pose.pose.position.x
			y = data.pose.pose.position.y
			print(" x = " + str(x) + "; y = " + str(y) + "  seq = " + str(data.header.seq))

			#print("seq: " + str(data.header.seq) + "; x = " + str(x) + " -> " + str(int(x / 0.05 + self.width/2)) )
			#print("seq: " + str(data.header.seq) + "; y = " + str(y) + " -> " + str(int(y / 0.05 + self.height/2)) )

			x_img = int((x + self.scaling) / self.resolution + self.width/2)
			y_img = int(y / self.resolution + self.height/2)

			# create a cross that points to the amcl pose
			self.image[x_img][y_img] = 150
			self.image[x_img+1][y_img] = 150
			self.image[x_img-1][y_img] = 150
			self.image[x_img][y_img+1] = 150
			self.image[x_img][y_img-1] = 150
			self.image[x_img+2][y_img] = 150
			self.image[x_img-2][y_img] = 150
			self.image[x_img][y_img+2] = 150
			self.image[x_img][y_img-2] = 150

	def write_map(self, msg):
		filename = os.path.abspath('..') + '/maps/map_trajectory.png'
		print ("Saving image with map and trajectory")
		cv2.imwrite(filename, self.image)
		return TrajectorySrvResponse(True)


if __name__ == '__main__':
	rospy.init_node('trajectory', anonymous=False)
	trajectory = Trajectory()
	rospy.spin()
