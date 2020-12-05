#!/usr/bin/env python3

# Import what we need.
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan


class Median_filter:
	def __init__(self):
		# Initialize subscriber
		self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

		# Initialize publishers
		self.median_filter_pub = rospy.Publisher('/median_filter', LaserScan, queue_size=1)


	def laser_callback(self, scan_msg):
		# Create a LaserScan message named "scan".
		output_list =[]
		for i in range(len(scan_msg)):
			if i == 0 or i == len(scan_msg):
				output_list.append(np.inf)
				pass
			else:
				_,m,_=sorted([scan_msg[i-1], scan_msg[i], scan_msg[i+1]])
				output_list.append(m)
		scan_msg.ranges = output_list
		self.median_filter_pub.publish(scan_msg)

if __name__ == '__main__':
	# Initialize the node
 	rospy.init_node('median_filter')
	Median_filter()

	# Keep running node
	rospy.spin()
