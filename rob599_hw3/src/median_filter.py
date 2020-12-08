#!/usr/bin/env python3

# Import what we need.
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan
from statistics import median

class Median_filter:
	def __init__(self):
		# Initialize subscriber
		self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

		# Initialize publishers
		self.median_filter_pub = rospy.Publisher('/median_filter', LaserScan, queue_size=1)

		# Acquire the median_filter_size parameter
		self.size = rospy.get_param("median_filter_size")


	def laser_callback(self, scan_msg):
		# Create a LaserScan message named "scan".
		output_list =[]
		# print(scan_msg)
		for i in range(len(scan_msg.ranges)):
			if i >= self.size and (i + self.size) <= len(scan_msg.ranges):
			 	output_list.append(median(scan_msg.ranges[(i -self.size):(i + self.size + 1)]))
			else:
				output_list.append(np.inf)

		# print("")
		# print(output_list)
		scan_msg.ranges = output_list
		self.median_filter_pub.publish(scan_msg)
		# print(scan_msg)

if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('median_filter')
	Median_filter()
	# val = rospy.get_param("filter_size")
	# print(type(val))
	# Keep running node
	rospy.spin()
