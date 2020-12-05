#!/usr/bin/env python

# Import what we need.
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan


class Noiser:
	def __init__(self):
		# Initialize subscriber
		self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

		# Initialize publishers
		self.median_filter_pub = rospy.Publisher('/median_filter', LaserScan, queue_size=1)


	def laser_callback(self, scan_msg, sigma=0.01):
		noise_list = []
		for i in range(len(scan_msg)):
			noise_list.append(scan_msg[i] + gauss(0.0,sigma))

		scan_msg.ranges = noise_list
		self.median_filter_pub.publish(scan_msg)




if __name__ == '__main__':
	# Initialize the node
 	rospy.init_node('noiser')
	Noiser()

	# Keep running node
	rospy.spin()
