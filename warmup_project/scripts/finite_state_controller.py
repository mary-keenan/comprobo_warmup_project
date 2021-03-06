#!/usr/bin/env python
""" This script makes a robot move in a square until it sees a wall,
then move in a square again once the wall disappears """

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rospy
import math


class finite_state_controller(object):


	def __init__(self):
		rospy.init_node('bumpty_dumpty')
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.rate = rospy.Rate(100)
		self.vel_msg = Twist()
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0
		self.wall_is_nearby = False # used to determine state transitions
		self.desired_heading = 0

		# set parameters
		self.base_angular_speed_for_wall = .2
		self.time_until_turn = 3.2 # seconds
		self.time_for_turn = 1.57 # seconds
		self.threshold_for_parallelism = .03 # how high the standard for parallelism is
		self.threshold_for_wall = 1 # how close the wall should be before the robot recognizes it
		self.min_number_of_points_for_wall = 20 # how many points need to be within the threshold for a wall to be recognized


	def run(self):
		# set up sensor processing for Wall Following behavior
		self.subscribe_to_laser_sensor()

		# initialize variables needed for moving forward in Square Dancing
		time_started_moving_straight = rospy.Time.now()
		time_stop_moving_straight = time_started_moving_straight + rospy.Duration(self.time_until_turn)

		while not rospy.is_shutdown():
			# move straight for self.time_until_turn amount of time unless wall is detected
			while rospy.Time.now() < time_stop_moving_straight and not self.wall_is_nearby and not rospy.is_shutdown():
				print "square dancing"
				self.vel_msg.linear.x = 1
				self.vel_msg.angular.z = 0
				self.publisher.publish(self.vel_msg)

			# if wall is detected, follow it until it's gone
			while self.wall_is_nearby and not rospy.is_shutdown():
				print "following wall"
				self.vel_msg.linear.x = .1
				self.vel_msg.angular.z = self.desired_heading # constantly being recalculated
				self.publisher.publish(self.vel_msg)

			# continue with behavior of Square Dancing state by turning
			print "square dancing"
			self.turn()
			self.rate.sleep()
			time_started_moving_straight = rospy.Time.now()
			time_stop_moving_straight = time_started_moving_straight + rospy.Duration(self.time_until_turn)		


	# DRIVE SQUARE
	def turn(self):
		""" turn for self.time_for_turn amount of time """
		time_started_turning = rospy.Time.now()
		time_stop_turning = time_started_turning + rospy.Duration(self.time_for_turn)
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = -1

		while rospy.Time.now() < time_stop_turning:
			self.publisher.publish(self.vel_msg)


	# PROCESS LASER SENSOR DATA TO DETERMINE STATE
	def subscribe_to_laser_sensor(self):
		rospy.Subscriber("/stable_scan", LaserScan, self.process_sensor_reading)


	def process_sensor_reading(self, data):
		""" performs check for state change and sets values for Wall Following """
		self.check_if_wall_nearby(data)
		if not self.check_if_parallel(data):
			self.set_heading(data)


	# FOLLOW WALL
	def check_if_wall_nearby(self, data):
		""" checks if a certain number of points fall within the specified distance for wall detection """
		possible_wall_hits = 0
		for distance in data.ranges:
			if distance > 0.0 and distance < self.threshold_for_wall:
				possible_wall_hits += 1
		if possible_wall_hits > self.min_number_of_points_for_wall:
			self.wall_is_nearby = True
			return
		self.wall_is_nearby = False


	def check_if_parallel(self, data):
		""" checks if the robot is within the threshold for parallelism (it doesn't necessarily need to be
			perfectly parallel, just within the specified range for error, self.threshold_for_parallelism)"""

		# figure out if it's closest to an object in front of it (vs to the side of it)
		front_angles = filter(lambda x: x != 0.0, data.ranges[-30:] + data.ranges[:30])
		left_angles = filter(lambda x: x != 0.0, data.ranges[60:120])
		right_angles = filter(lambda x: x != 0.0, data.ranges[-120:-60])

		# handle case where there are no readings because there are no objects close by in that direction
		if len(front_angles) != 0:
			if len(left_angles) != 0 and len(right_angles) != 0:
				front_avg = sum(front_angles)/len(front_angles)
				left_avg = sum(left_angles)/len(left_angles)
				right_avg = sum(right_angles)/len(right_angles)

				if front_avg < left_avg and front_avg < right_avg:
					return False

		# define angle pairs we want to check for each side
		left_pairs = [(45,135),(60,120),(70,110),(75,105),(80,100),(85,95)]
		right_pairs = [(225,315),(240,300),(250,290),(255,285),(260,280),(275,285)]

		# initalize values
		left_difference = 0
		right_difference = 0
		left_count = 0 # number of non-zero pairs
		right_count = 0

		# calculate the average difference between the angle pairs (should be 0 or almost 0 if parallel)
		for pair in left_pairs:
			left_distance = data.ranges[pair[0]]
			right_distance = data.ranges[pair[1]]
			if left_distance != 0 and right_distance != 0:
				left_difference += abs(left_distance - right_distance)
				left_count += 1

		for pair in right_pairs:
			left_distance = data.ranges[pair[0]]
			right_distance = data.ranges[pair[1]]
			if left_distance != 0.0 and right_distance != 0.0:
				right_difference += abs(left_distance - right_distance)
				right_count += 1

		# if the average difference for either the left or the right is below the specified threshold, the robot is parallel to the wall
		if (left_count > 0 and (left_difference/left_count) < self.threshold_for_parallelism) or (right_count > 0 and (right_difference/right_count) < self.threshold_for_parallelism):
			self.desired_heading = 0
			return True
		return False


	def set_heading(self, data):
		""" determines which of the four quadrants (front left, back left, back right, and front right)
			are closest to the wall and adjusts direction based on that """
		
		# initialize quadrant lists
		front_left_quadrant = data.ranges[0:90]
		front_left_values = [10] # just so the length can't be zero when finding average
		back_left_quadrant = data.ranges[90:180]
		back_left_values = [10]
		back_right_quadrant = data.ranges[180:270]
		back_right_values = [10]
		front_right_quadrant = data.ranges[270:360]
		front_right_values = [10]

		# find the avg value for each quadrant
		for front_left_value in front_left_quadrant:
			if front_left_value != 0.0:
				front_left_values.append(front_left_value)
		front_left_avg = sum(front_left_values)/len(front_left_values)

		for back_left_value in back_left_quadrant:
			if back_left_value != 0.0:
				back_left_values.append(back_left_value)
		back_left_avg = sum(back_left_values)/len(back_left_values)

		for front_right_value in front_right_quadrant:
			if front_right_value != 0.0:
				front_right_values.append(front_right_value)
		front_right_avg = sum(front_right_values)/len(front_right_values)

		for back_right_value in back_right_quadrant:
			if back_right_value != 0.0:
				back_right_values.append(back_right_value)
		back_right_avg = sum(back_right_values)/len(back_right_values)

		# determine direction and speed of heading based on differences between quadrant averages
		if front_left_avg < front_right_avg and front_left_avg < back_left_avg: # robot is headed towards the wall to the left, so it should go right
			self.desired_heading = -self.base_angular_speed_for_wall * (back_left_avg - front_left_avg)
		elif front_left_avg < front_right_avg and front_left_avg > back_left_avg: # robot is headed away from the wall to the right, it should go left
			self.desired_heading = self.base_angular_speed_for_wall * (front_left_avg - back_left_avg)
		elif front_left_avg > front_right_avg and front_right_avg < back_right_avg: # robot is headed towards the wall to the right, so it should go left
			self.desired_heading = self.base_angular_speed_for_wall * (back_right_avg - front_right_avg)
		elif front_left_avg > front_right_avg and front_right_avg > back_right_avg: # robot is headed away from the wall to the left, it should go right
			self.desired_heading = -self.base_angular_speed_for_wall * (front_right_avg - back_right_avg)



if __name__ == '__main__':
	node = finite_state_controller()
	node.run()