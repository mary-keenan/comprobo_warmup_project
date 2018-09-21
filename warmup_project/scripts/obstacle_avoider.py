#!/usr/bin/env python
""" This script makes a robot avoid obstacles"""

from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import rospy
import math


class obstacle_avoider(object):

	def __init__(self):
		rospy.init_node('bumpty_dumpty')
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.rate = rospy.Rate(100)
		self.vel_msg = Twist()
		self.vel_msg.linear.x = .1
		self.vel_msg.angular.z = 0
		self.position_x = 0
		self.position_y = 0
		self.angle = 0

		# set parameters
		self.base_angular_speed = .2
		self.base_linear_speed = .1


	def run(self):
		self.subscribe_to_laser_sensor()
		self.subscribe_to_position()

		while not rospy.is_shutdown():
			# publish velocity
			self.publisher.publish(self.vel_msg)
			self.rate.sleep()


	def subscribe_to_laser_sensor(self):
		rospy.Subscriber("/stable_scan", LaserScan, self.process_sensor_reading)


	def process_sensor_reading(self, data):
		# convert angle/distance to x/y values
		points_list = []
		for angle in range(360):
			distance = data.ranges[angle]
			if distance != 0.0:
				point = self.calculate_point_position(angle, 10 - distance) # closer should give it more magnitude
				points_list.append(point)

		# sum x and y values
		x_sum = 100 # weight it so it wants to go forward at all costs
		y_sum = 0
		for point in points_list:
			x_sum += point.x
			y_sum += point.y

		# find the resulting angle/magnitude
		angle, magnitude = self.calculate_velocity(x_sum, y_sum)
		print "angle: " + str(angle) + " magnitude: " + str(magnitude)

		# turn the robot away from the angle, angular velocity depends on magnitude
		if angle > 90 and angle < 270:
			angle_proportion = math.cos(math.radians(angle - 90))
			print angle_proportion
			magnitude_proportion = (magnitude ** (.125))
			self.vel_msg.angular.z = self.base_angular_speed * angle_proportion * magnitude_proportion
			print magnitude_proportion


	def calculate_point_position(self, angle, distance):
		fixed_frame_point_angle = self.angle + math.radians(angle) # angle of robot + angle of point
		x_position = self.position_x + math.cos(fixed_frame_point_angle) * distance
		y_position = self.position_y + math.sin(fixed_frame_point_angle) * distance
		return Vector3(x = x_position, y = y_position)


	def calculate_velocity(self, x, y):
		if y == 0:
			y = 0.00000001 # so we don't get division by zero complaints
		angle = math.degrees(math.atan(x/y)) + 180 # because we want to go the opposite direction
		magnitude = math.sqrt(x**2 + y**2)
		return (angle, magnitude)


	def subscribe_to_position(self):
		rospy.Subscriber("/odom", Odometry, self.update_position)


	def update_position(self, odom):
		""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
		pose = odom.pose.pose
		orientation_tuple = (pose.orientation.x,
							 pose.orientation.y,
							 pose.orientation.z,
							 pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		self.position_x = pose.position.x
		self.position_y = pose.position.y
		self.angle = angles[2]

if __name__ == '__main__':
	node = obstacle_avoider()
	node.run()
