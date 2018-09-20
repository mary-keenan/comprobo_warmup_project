#!/usr/bin/env python
""" This script makes a robot follow a person at some specified distance """
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from visualization_msgs.msg import Marker
import rospy
import math
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix


class person_follower(object):

	def __init__(self):
		rospy.init_node('bumpty_dumpty')
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.rate = rospy.Rate(100)
		self.vel_msg = Twist()
		self.marker = Marker(scale=Vector3(x=1,y=1))
		self.marker.color.a = 1
		self.marker.color.r = 1
		self.marker_publisher = rospy.Publisher('person_marker', Marker, queue_size = 10)
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0
		self.position_x = 0
		self.position_y = 0
		self.angle = 0

		# set parameters
		self.allowed_dist_from_person = .5
		self.base_angular_speed = .005
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
		# First, find angle at which closest big object is at
		size_of_box = 20
		moving_range_indices = (0, size_of_box)
		min_avg_value = 10
		min_avg_center_index = None
		values = []
		for value in data.ranges:
			if value == 0:
				values.append(10)
			else:
				values.append(value)

		for moving_range_index in range(360/size_of_box):
			start_index = moving_range_index * size_of_box
			end_index = (moving_range_index + 1) * size_of_box
			moving_range_values = values[start_index:end_index]
			moving_range_avg = sum(moving_range_values) / len(moving_range_values)
			if moving_range_avg > 0 and min_avg_value >= moving_range_avg:
				min_avg_value = moving_range_avg
				min_avg_center_index = (start_index + end_index)/2

		print "angle: " + str(min_avg_center_index)
		if min_avg_center_index > 180: # the person is on the left
			min_avg_center_index = -(min_avg_center_index - 180) # covers case where person is to the right of the robot

		# robots turns / moves depending on where the person is
		if min_avg_value < self.allowed_dist_from_person: # the robot is too close=
			self.vel_msg.angular.z = -self.base_angular_speed * min_avg_center_index
			self.vel_msg.linear.x = -self.base_linear_speed
		else: # the robot needs to catch up
			self.vel_msg.angular.z = self.base_angular_speed * min_avg_center_index
			self.vel_msg.linear.x = self.base_linear_speed

		print "dist: " + str(min_avg_value)
		print "ang: " + str(self.vel_msg.angular.z)
		print "lin: " + str(self.vel_msg.linear.x)

		# visualize person
		marker_points = []
		for angle in range(min_avg_center_index - size_of_box / 2, min_avg_center_index + size_of_box / 2):
			point = self.calculate_point_position(angle, values[angle])
			marker_points.append(point)
		self.marker_publisher.publish(self.marker)


	def calculate_point_position(self, angle, distance):
		fixed_frame_point_angle = self.angle + angle # angle of robot + angle of point
		x_position = self.position_x + math.cos(fixed_frame_point_angle) * distance
		y_position = self.position_y + math.sin(fixed_frame_point_angle) * distance
		return Vector3(x = x_position, y = y_position)


	def subscribe_to_position(self):
		rospy.Subscriber("/pose", Pose, self.update_position)


	def update_position(self, pose):
		""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
		orientation_tuple = (pose.orientation.x,
							 pose.orientation.y,
							 pose.orientation.z,
							 pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		self.position_x = pose.position.x
		self.position_y = pose.position.y
		self.angle = angles[2]



if __name__ == '__main__':
	node = person_follower()
	node.run()