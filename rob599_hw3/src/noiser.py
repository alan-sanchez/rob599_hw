#!/usr/bin/env python

# Import what we need.
import rospy
import numpy as np
from random import gauss

from sensor_msgs.msg import LaserScan


class Noiser:
	def __init__(self):
		# Initialize subscriber
		self.laser_sub = rospy.Subscriber('/median_filter', LaserScan, self.laser_callback)

		# Initialize publishers
		self.median_filter_pub = rospy.Publisher('/noiser', LaserScan, queue_size=1)

		# Acquire the median_filter_size parameter
		self.var = rospy.get_param("laser_noise_variance")

	def laser_callback(self, scan_msg):
		noise_list = []
		for i in range(len(scan_msg.ranges)):
			noise_list.append(scan_msg.ranges[i] + gauss(0.0,self.var))

		scan_msg.ranges = noise_list
		self.median_filter_pub.publish(scan_msg)




if __name__ == '__main__':
	# Initialize the node
 	rospy.init_node('noiser')
	Noiser()

	# Keep running node
	rospy.spin()
