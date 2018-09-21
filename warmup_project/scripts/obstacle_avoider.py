#!/usr/bin/env python
""" This script makes a robot avoid obstacles"""

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
import rospy
import math


class obstacle_avoider(object):

	def __init__(self):
		""" initializes the obsctacle_avoider object """
		rospy.init_node('bumpty_dumpty')
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.rate = rospy.Rate(100)
		self.vel_msg = Twist()
		self.vel_msg.linear.x = .1
		self.vel_msg.angular.z = 0
		# set up the variables that represent the Neato's position -- TODO, might not need?
		self.position_x = 0
		self.position_y = 0
		self.angle = 0

		# set parameters
		self.base_angular_speed = .2


	def run(self):
		""" starts the obstacle-avoiding functionality """
		self.subscribe_to_laser_sensor()
		self.subscribe_to_bump_sensor()

		while not rospy.is_shutdown():
			# publish velocity
			self.publisher.publish(self.vel_msg)
			self.rate.sleep()


	def subscribe_to_laser_sensor(self):
		""" starts subcribing to laser topic with a callback for processing the data"""
		rospy.Subscriber("/stable_scan", LaserScan, self.process_sensor_reading)


	def process_sensor_reading(self, data):
		""" converts raw data into heading for Neato """

		# convert angle/distance to x/y values
		points_list = []
		for angle in range(360):
			distance = data.ranges[angle]
			if distance != 0.0:
				point = self.calculate_point_position(angle, 10 - distance) # closer points should have more magnitude
				points_list.append(point)

		# sum x and y values to get net force in x and y directions
		x_sum = 100 # weight it so it wants to go forward more
		y_sum = 0
		for point in points_list:
			x_sum += point.x
			y_sum += point.y

		# find the resulting angle/magnitude of the summed forces
		angle, magnitude = self.calculate_velocity(x_sum, y_sum)

		# if the force is pushing forwards (<90, >270), the robot doesn't need to change its behavior because there's no obvious obstacle in its path yet;
		# if the force is pushing backwards (>90, <270), however, it means there's something in front of the robot and it needs to turn
		if angle > 90 or angle < 270:
			angle_proportion = math.sin(math.radians(angle))
			magnitude_proportion = (magnitude ** (.125))
			self.vel_msg.angular.z = self.base_angular_speed * angle_proportion * magnitude_proportion


	def calculate_point_position(self, angle, distance):
		""" converts from polar coordinates to cartesian """
		angle_in_radians = math.radians(angle + 180) 
		x_position = math.cos(angle_in_radians) * distance
		y_position = math.sin(angle_in_radians) * distance
		return Vector3(x = x_position, y = y_position)


	def calculate_velocity(self, x, y):
		""" converts from cartesian coordinates to polar """
		if y == 0:
			y = 0.00000001 # so we don't get division by zero complaints
		angle = math.degrees(math.atan(x/y))
		magnitude = math.sqrt(x**2 + y**2)
		return (angle, magnitude)


	def subscribe_to_bump_sensor(self):
		""" starts subcribing to bump topic with a callback for processing the data"""
		rospy.Subscriber("/bump", Bump, self.stop_movement)


	def stop_movement(self, msg):
		""" makes the Neato stop moving if it bumps into anything (presumably a wall) """
		if msg.leftFront == 1 or msg.leftSide == 1 or msg.rightFront == 1 or msg.rightSide == 1:
			self.vel_msg.linear.x = 0
			self.vel_msg.angular.z = 0



if __name__ == '__main__':
	node = obstacle_avoider()
	node.run()
