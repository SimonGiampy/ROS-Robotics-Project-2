#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
import os

class Trajectory:
	def __init__(self):
		self.cwd = os.getcwd()  #script path 
		rospy.Subscriber("/map", OccupancyGrid, self.callback_map)
		rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.callback_trajectory)
		self.map_ready = False
	
	def callback_map(self, data):
		self.width = data.info.width
		self.height = data.info.height
		size = (self.width , self.height)
		self.image = np.zeros(size)

		i = 0
		for pixel in data.data:  #copy image
			self.image[i% self.width][int(i/ self.width)] = pixel
			i+=1

		self.image[self.image == -1] = 50   #grey
		self.image[self.image == 0] = 255   #white
		self.image[self.image == 100] = 0   #black

		self.map_ready = True

	def callback_trajectory(self, data):
		if(self.map_ready):
			x = data.pose.pose.position.x
			y = data.pose.pose.position.y
			print(" x = " + str(x) + "; y = " + str(y) + "  seq = " + str(data.header.seq))

			#print("seq: " + str(data.header.seq) + "; x = " + str(x) + " -> " + str(int(x / 0.05 + self.width/2)) )
			#print("seq: " + str(data.header.seq) + "; y = " + str(y) + " -> " + str(int(y / 0.05 + self.height/2)) )
			
			#self.image[(x+27.2)*0.05][(y+16)*0.05] = 50 
			x_img = int((x + 5.6) / 0.05 + self.width/2)
			y_img = int(y / 0.05 + self.height/2)

			self.image[x_img][y_img] = 150
			self.image[x_img+1][y_img] = 150
			self.image[x_img-1][y_img] = 150
			self.image[x_img][y_img+1] = 150
			self.image[x_img][y_img-1] = 150
			self.image[x_img+2][y_img] = 150
			self.image[x_img-2][y_img] = 150
			self.image[x_img][y_img+2] = 150
			self.image[x_img][y_img-2] = 150





			if(data.header.seq == 20):
				filename = self.cwd + '/test_pixel3.png'
				print ("saving image")
				cv2.imwrite (filename,self.image)


if __name__ == '__main__':
	rospy.init_node('depth_saver', anonymous=False)
	trajectory = Trajectory()
	rospy.spin()