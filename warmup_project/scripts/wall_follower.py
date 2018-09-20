#!/usr/bin/env python
""" This script makes a robot follow a wall in parallel """
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import rospy
import math
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix


class wall_follower(object):

	def __init__(self):
		rospy.init_node('bumpty_dumpty')
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.rate = rospy.Rate(100)
		self.vel_msg = Twist()
		self.marker = Marker(scale = Vector3(x = .05, y = .05), type = 8)
		self.marker.color.a = 1
		self.marker.color.r = 1
		self.marker.header.frame_id = "odom"
		self.marker_publisher = rospy.Publisher('wall_marker', Marker, queue_size = 10)
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0
		self.position_x = 0
		self.position_y = 0
		self.angle = 0

		# set parameters
		self.threshold_for_parallelism = .03 # how high the standard for parallelism is
		self.threshold_for_wall = 1 # how close the wall should be before the robot starts turning
		self.base_angular_speed = .3 # used for proportional control
		self.forward_speed = .1 # default linear speed

	def run(self):
		self.subscribe_to_position()
		self.subscribe_to_laser_sensor()
		self.subscribe_to_bump_sensor()
		self.vel_msg.linear.x = self.forward_speed 

		while not rospy.is_shutdown():
			# publish velocity
			self.publisher.publish(self.vel_msg)
			self.rate.sleep()


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


	def subscribe_to_laser_sensor(self):
		rospy.Subscriber("/stable_scan", LaserScan, self.process_sensor_reading)


	def process_sensor_reading(self, data):
		# First, figure out if there is a wall nearby
		if not self.check_if_wall_nearby(data):
			return

		# Second, check if the robot is already parallel to the wall
		if self.check_if_parallel(data):
			return

		# Third, figure out the angle between the robot's heading and the wall
		self.set_heading(data)


	def check_if_wall_nearby(self, data):
		possible_wall_hits = 0
		wall_points = []
		for angle in range(360):
			distance = data.ranges[angle]
			if distance > 0.0 and distance < self.threshold_for_wall:
				possible_wall_hits += 1
				point = self.calculate_point_position(angle, distance)
				wall_points.append(point)
		if possible_wall_hits < 20:
			return False

		self.marker.points = wall_points
		self.marker_publisher.publish(self.marker)
		return True		


	def calculate_point_position(self, angle, distance):
		fixed_frame_point_angle = self.angle + math.radians(angle) # angle of robot + angle of point
		x_position = self.position_x + math.cos(fixed_frame_point_angle) * distance
		y_position = self.position_y + math.sin(fixed_frame_point_angle) * distance
		return Vector3(x = x_position, y = y_position)


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
		front_left_quadrant = data.ranges[0:90]
		front_left_values = [10] # just so the length can't be zero when finding average
		back_left_quadrant = data.ranges[90:180]
		back_left_values = [10]
		back_right_quadrant = data.ranges[180:270]
		back_right_values = [10]
		front_right_quadrant = data.ranges[270:360]
		front_right_values = [10]

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

		if front_left_avg < front_right_avg and front_left_avg < back_left_avg: # robot is headed towards the wall to the left, so it should go right
			self.vel_msg.angular.z = -self.base_angular_speed * (back_left_avg - front_left_avg)
		elif front_left_avg < front_right_avg and front_left_avg > back_left_avg: # robot is headed away from the wall to the right, it should go left
			self.vel_msg.angular.z = self.base_angular_speed * (front_left_avg - back_left_avg)
		elif front_left_avg > front_right_avg and front_right_avg < back_right_avg: # robot is headed towards the wall to the right, so it should go left
			self.vel_msg.angular.z = self.base_angular_speed * (back_right_avg - front_right_avg)
		elif front_left_avg > front_right_avg and front_right_avg > back_right_avg: # robot is headed away from the wall to the left, it should go right
			self.vel_msg.angular.z = -self.base_angular_speed * (front_right_avg - back_right_avg)


	def subscribe_to_bump_sensor(self):
		rospy.Subscriber("/bump", Bump, self.stop_movement)

	def stop_movement(self, msg):
		if msg.leftFront == 1 or msg.leftSide == 1 or msg.rightFront == 1 or msg.rightSide == 1:
			self.vel_msg.linear.x = 0
			self.vel_msg.angular.z = 0

if __name__ == '__main__':
	node = wall_follower()
	node.run()

