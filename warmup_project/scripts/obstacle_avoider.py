#!/usr/bin/env python
""" This script makes a robot avoid obstacles"""

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rospy
import math


class obstacle_avoider(object):

	def __init__(self):
		rospy.init_node('bumpty_dumpty')
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.rate = rospy.Rate(100)
		self.vel_msg = Twist()
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0

		# set parameters
		self.base_angular_speed = .2
		self.base_linear_speed = .1


	def run(self):
		self.subscribe_to_laser_sensor()
		self.vel_msg.linear.x = self.forward_speed 

		while not rospy.is_shutdown():
			# publish velocity
			self.publisher.publish(self.vel_msg)
			self.rate.sleep()


	def subscribe_to_laser_sensor(self):
		rospy.Subscriber("/stable_scan", LaserScan, self.process_sensor_reading)


	def process_sensor_reading(self, data):
		# convert angle/distance to x/y values
		# sum x and y values
		# find the resulting angle/magnitude
		# turn the robot away from the angle, angular velocity depends on magnitude







if __name__ == '__main__':
	node = obstacle_avoider()
	node.run()
