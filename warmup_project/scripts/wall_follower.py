#!/usr/bin/env python
""" This script makes a robot follow a wall in parallel """

import rospy
from geometry_msgs.msg import Twist
import Math


class wall_follower(object):

	def __init__(self):
		ropsy.init_node('bumpty_dumpty')
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.rate = rospy.Rate(10)
		self.vel_msg = Twist()
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0

		# set parameters
		self threshold_for_parallelism = 1


	def run(self):
		self.subscribe_to_laser_sensor()
		self.vel_msg.linear.x = .1

		while not rospy.is_shutdown():
			# publish velocity
			self.publisher.publish(self.vel_msg)
			self.rate.sleep()


	def subscribe_to_laser_sensor(self):
		rospy.Subscriber("/stable_scan", LaserScan, self.process_sensor_reading)


	def process_sensor_reading(self, data):
		# First, figure out if there is a wall nearby
		if !self.check_if_wall_nearby(data):
			return

		# Second, check if the robot is already parallel to the wall
		if self.check_if_parallel(data):
			return

		# Third, figure out the angle between the robot's heading and the wall
		self.set_heading(data)


	def check_if_wall_nearby(self, data):
		possible_wall_hits = 0
		for value in data.ranges:
			if value is not 0 and value < .5: # TODO magic numbers; we don't include 0s because they don't mean anything
				possible_wall_hits++
		if possible_wall_hits < 10:
			return False
		return True		


	def check_if_parallel(self, data):
		# define angle pairs we want to check for each side
		left_pairs = [(45,135),(60,120),(70,110),(75,105),(80,100),(85,95)]
		right_pairs = [(225,315),(240,300),(250,290),(255,285),(260,280),(275,285)]

		# initalize values
		left_difference = 0
		right_difference = 0

		# calculate the total difference between the angle pairs (should be 0 or almost 0 if parallel)
		for pair in left_pairs:
			left_distance = data.ranges[pair[0]]
			right_distance = data.ranges[pair[1]]
			if left_distance != 0 and right_distance != 0:
				left_difference += abs(left_distance - right_distance) # TODO is it Math.abs() ?

		for pair in right_pairs:
			left_distance = data.ranges[pair[0]]
			right_distance = data.ranges[pair[1]]
			if left_distance != 0 and right_distance != 0:
				right_difference += abs(left_distance - right_distance)

		# determine if the robot is parallel to the wall given a threshold for error/variation
		if left_difference < self.threshold_for_parallelism or right_difference < self.threshold_for_parallelism:
			self.vel_msg.angular.z = 0
			return True
		return False


	def set_heading(self, data):
		forward_pairs = [(45,-45),(40,-40),(35,-35),(30,-30),(25,-25)]
		calculated_angles = []
		for pair in forward_pairs:
			left_distance = data.ranges[pair[0]]
			right_distance = data.ranges[pair[1]]
			if left_distance != 0 and right_distance != 0:
				hypotenuse = Math.sqrt(left_distance ** 2 + right_distance ** 2)
				left_desired_angle = 180 - (Math.degrees(Math.acos(left_distance / hypotenuse)) + pair[0])
				right_desired_angle = 180 - (Math.degrees(Math.acos(right_distance / hypotenuse)) + pair[0])
				if left_desired_angle < right_desired_angle:
					#turn right
				else: # this catches the case where the angle is 90 degrees
					#turn left



if __name__ == '__main__':
	node = wall_follower()
	node.run()