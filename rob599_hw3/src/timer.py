#!/usr/bin/env python3

# Import what we need.
import rospy
import time
import pickle
import os
import numpy as np

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from math import pi

class Timer:
	def __init__(self):
		# Initialize publishers
		self.laserscan_pub = rospy.Publisher('/scan', LaserScan, queue_size=5)

		# Acquire the median_filter_size parameter
		self.topic = rospy.get_param("topic_scan")

		# Initialize subscribers
		self.laserscan_sub	   = rospy.Subscriber(self.topic, LaserScan, self.laser_callback)

		# Initialize scan message
		self.scan = LaserScan()
		self.scan.angle_min = -30 * (pi/180)
		self.scan.angle_max =  30 * (pi/180)
		self.scan.angle_increment = 1 * (pi/180)
		self.scan.range_min = 0
		self.scan.range_max = 10
		scan_len = int((self.scan.angle_max - self.scan.angle_min)/self.scan.angle_increment)
		self.scan.ranges = np.random.uniform(low=self.scan.angle_min, high=self.scan.angle_max, size=(scan_len,))

		# Get the directory path of the script file. This pickle file will be saved here
		self.dir_path = os.path.dirname(os.path.realpath(__file__))

		# Intialize start time
		self.start_time = None

		# Intialize end time
		self.end_time = None

		# Intialize list of times
		self.list_times = []

	def publisher_scan_msg(self):
		# Get time before the scan is published
		self.start_time = time.time()
		self.laserscan_pub.publish(self.scan)

	def laser_callback(self,msg):
		# Get the time for when subscriber gets
		self.end_time = time.time()
		self.list_times.append(self.end_time - self.start_time)

	def average_time(self):
		avg = sum(self.list_times)/100
		print("Average Time: {0}".format(avg))
		with open(self.dir_path + '/time.pickle', 'wb') as handle:
				pickle.dump(avg, handle, protocol=None)
				print("The data structure has been saved")
		with open('time.pickle', 'rb') as handle:
				time = pickle.load(handle)
				print(time)
		del self.list_times[:]



if __name__ == '__main__':
	# Initialize the node
	rospy.init_node('timer')

	func = Timer()

	# Manage the rate.
	rate = rospy.Rate(60)

	for i in range(100):
		func.publisher_scan_msg()
		rate.sleep()

	func.average_time()
