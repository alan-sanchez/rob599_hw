#!/usr/bin/env python

import math
import rospy
import actionlib

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

class Wall_angle:
	def __init__(self):
		# Set up a subscriber the subscribes to a topic called "base_scan" with
		# a message type LaserScan. This will be sent to the laser_callback function.
		self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback, queue_size = 1)

		# Initialize publishers
		self.sphere_marker_pub  = rospy.Publisher('sphere_marker',	 Marker, 	queue_size=10)
		self.text_marker_pub  = rospy.Publisher('text_angle_marker',	 Marker, 	queue_size=10)


	# laser_callback called every time we get a LaserScan message from ROS.
	def laser_callback(self, msg):
		center_index = int(round(len(msg.ranges)/2.0))
		r1 = msg.ranges[center_index]

		num_incr = 50
		right_range = msg.ranges[center_index + num_incr]
		left_range  = msg.ranges[center_index - num_incr]

		if right_range > left_range:
			r2 = right_range
		else:
			r2 = left_range

		theta = msg.angle_increment*num_incr
		# law of cosines
		d = math.sqrt((r2*r2) + (r1*r1) - 2*r1*r2*math.cos(theta))
		# law of sines
		alpha = math.asin( r1*math.sin(theta)/d )
		phi = abs(alpha*180/math.pi - 90)
		# print(phi)

		r3 = 1
		x_pos = r1
		if right_range > left_range:
			tip_x = (-r3 * math.cos((phi)*math.pi/180)) - x_pos
		else:
			tip_x = r3 * math.cos((phi)*math.pi/180) - x_pos

		tip_y = r3 * math.sin(phi*math.pi/180)
		# print(tip_x, tip_y)
		self.sphere_marker(msg, x_pos,tip_x,tip_y)
		self.text_marker(msg,phi, x_pos)


	def sphere_marker(self,msg,x_pos,tip_x,tip_y):
		sphere = Marker()
		sphere.header	 = msg.header
		sphere.type 	 = Marker.SPHERE
		sphere.action  = Marker.ADD
		sphere.id 	 = 1
		sphere.scale.x = .1
		sphere.scale.y = .1
		sphere.scale.z = .1
		sphere.color.a = 1
		sphere.color.r = 1
		sphere.color.g = 0
		sphere.color.b = 0
		sphere.pose.position.x = x_pos
		sphere.pose.position.y = 0
		sphere.pose.position.z = 0
		self.sphere_marker_pub.publish(sphere)

	def text_marker(self,msg,phi,x_pos):
		text = Marker()
		text.header	 = msg.header
		text.type 	 = Marker.TEXT_VIEW_FACING
		text.action  = Marker.ADD
		text.id 	 = 1
		text.text 	 = '{:2.1f} degrees'.format(phi)
		text.scale.z = 0.3
		text.color.a = 1
		text.color.r = 1
		text.color.g = 1
		text.color.b = 1
		text.pose.position.x = x_pos
		text.pose.position.y = 0
		text.pose.position.z = -.1
		self.text_marker_pub.publish(text)


if __name__ == '__main__':
	rospy.init_node('wall_angle')
	Wall_angle()
	rospy.spin()
