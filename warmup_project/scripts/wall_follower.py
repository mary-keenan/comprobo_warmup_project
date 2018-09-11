#!/usr/bin/env python
""" This script makes a robot follow a wall in parallel """
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
import rospy
import math


class wall_follower(object):

	def __init__(self):
		rospy.init_node('bumpty_dumpty')
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.rate = rospy.Rate(100)
		self.vel_msg = Twist()
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0

		# set parameters
		self.threshold_for_parallelism = .03
		self.threshold_for_wall = 1
		self.angular_speed = .002
		self.forward_speed = .1
		self.is_parallel = False


	def run(self):
		self.subscribe_to_laser_sensor()
		self.subscribe_to_bump_sensor()
		self.vel_msg.linear.x = self.forward_speed 

		while not rospy.is_shutdown():
			# publish velocity
			self.publisher.publish(self.vel_msg)
			self.rate.sleep()


	def subscribe_to_laser_sensor(self):
		rospy.Subscriber("/stable_scan", LaserScan, self.process_sensor_reading)


	def process_sensor_reading(self, data):
		if self.is_parallel:
			return

		# First, figure out if there is a wall nearby
		if not self.check_if_wall_nearby(data):
			return

		# Second, check if the robot is already parallel to the wall
		if self.check_if_parallel(data):
			self.is_parallel = True
			return

		# Third, figure out the angle between the robot's heading and the wall
		self.set_heading(data)


	def check_if_wall_nearby(self, data):
		possible_wall_hits = 0
		for value in data.ranges:
			if value > 0.0 and value < self.threshold_for_wall: # TODO magic numbers
				possible_wall_hits += 1 
		if possible_wall_hits < 20:
			return False
		return True		


	def check_if_parallel(self, data):
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
			self.vel_msg.angular.z = 0
			return True
		return False


	def set_heading(self, data):
		forward_pairs = []
		for i in range(35, 55):
			forward_pairs += [(i, 360 - i)]

		calculated_angles = []
		for pair in forward_pairs:
			left_distance = data.ranges[pair[0]]
			right_distance = data.ranges[pair[1]]
			if left_distance != 0 and right_distance != 0:
				hypotenuse = math.sqrt(left_distance ** 2 + right_distance ** 2)
				left_desired_angle = math.degrees(math.acos(left_distance / hypotenuse)) + pair[0]
				right_desired_angle = math.degrees(math.acos(right_distance / hypotenuse)) + (360 - pair[1])
				
				if left_desired_angle > right_desired_angle:
					calculated_angles += [-right_desired_angle]
				else: # handles edge case of incoming angle of 90 degrees (it will just turn left)
					calculated_angles += [left_desired_angle]
		if len(calculated_angles) > 0:
			avg_desired_angle = sum(calculated_angles)/len(calculated_angles)
			self.vel_msg.angular.z = self.angular_speed * avg_desired_angle
			#print "average angle: " + str(avg_desired_angle)


	def subscribe_to_bump_sensor(self):
		rospy.Subscriber("/bump", Bump, self.stop_movement)

	def stop_movement(self, msg):
		if msg.leftFront == 1 or msg.leftSide == 1 or msg.rightFront == 1 or msg.rightSide == 1:
			self.vel_msg.linear.x = 0
			self.vel_msg.angular.z = 0

if __name__ == '__main__':
	node = wall_follower()
	node.run()