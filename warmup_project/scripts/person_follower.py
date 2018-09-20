#!/usr/bin/env python
""" This script makes a robot follow a person at some specified distance """
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
import rospy
import math
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix


class person_follower(object):

	def __init__(self):
		rospy.init_node('bumpty_dumpty')
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.rate = rospy.Rate(1)
		self.vel_msg = Twist()
		self.marker = Marker(scale=Vector3(x=.05,y=.05),type = 8) # type 8 is points
		self.marker.color.a = 1
		self.marker.color.r = 1
		self.marker.header.frame_id = "odom"
		self.marker_publisher = rospy.Publisher('person_marker', Marker, queue_size = 10)
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0
		self.position_x = 0
		self.position_y = 0
		self.angle = 0
		self.last_angle = 0

		# set parameters
		self.allowed_dist_from_person = .6
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

		angle_of_legs, distance_from_legs = self.detect_legs(data)
		print "angle: " + str(angle_of_legs) + " distance: " + str(distance_from_legs)

		# keeps it from spazzing too much when a point jumps around by slowing dramatic turns
		difference_from_last_angle = self.last_angle - angle_of_legs
		if abs(difference_from_last_angle) > 20:
			self.last_angle = angle_of_legs + difference_from_last_angle/2
			return
		self.last_angle = angle_of_legs

		# visualize legs of person the robot is detecting and hopefully following
		self.visualize_legs(data, angle_of_legs)
		
		if self.check_if_appropriate_distance_from_person(distance_from_legs):
			return

		if angle_of_legs > -15 and angle_of_legs < 15: # straight enough
			self.vel_msg.angular.z = 0
		elif angle_of_legs < 180: # to the left
			self.vel_msg.angular.z = self.base_angular_speed * angle_of_legs
		else: # to the right
			self.vel_msg.angular.z = - self.base_angular_speed * (360 - angle_of_legs)

		distance_from_ideal_distance = distance_from_legs - self.allowed_dist_from_person
		
		if angle_of_legs > 90 and angle_of_legs < 270:
			self.vel_msg.linear.x = - self.base_linear_speed# * math.sqrt(distance_from_ideal_distance)
			self.vel_msg.angular.z *= -1
		else: 
			self.vel_msg.linear.x = self.base_linear_speed# * math.sqrt(distance_from_ideal_distance)


	def detect_legs(self, data):
		# weight it so farther away objects have less of a difference than closer ones
		distances = []
		for angle in range(360):
			distance = data.ranges[angle]
			if distance == 0.0:
				distances.append(10)
			else:
				distances.append(distance)
		distances += distances[1:5] # length of 365, with repeat of 1-5 at end

		# find moving averages
		moving_averages = [] # length of 390, with repeat of 1
		for angle in range(360):
			moving_range = distances[angle : angle + 5]
			moving_avg = sum(moving_range) / len(moving_range)
			moving_averages.append(moving_avg)
		moving_averages += [moving_averages[1]]

		differences_in_averages = [] # length of 390, with repeat of 1-30
		for angle in range(360):
			differences_in_averages.append(moving_averages[angle + 1] - moving_averages[angle])
		differences_in_averages += differences_in_averages[1:30]

		# find biggest negative then positive dip, within maybe 30 degrees
		max_difference_angle = 0
		max_difference = 0
		for angle in range(360):
			difference = differences_in_averages[angle]
			if difference < -1: # new angle is closer by .5m+, so search for farthest angle in range
				max_change_within_range = max(differences_in_averages[angle + 1:angle + 30])
				if max_change_within_range > 1: # matching angle is closer by .5m+
					net_difference = abs(difference) + max_change_within_range
					if net_difference > max_difference:
						max_difference_angle = angle
						max_difference = net_difference

		return (max_difference_angle, moving_averages[max_difference_angle + 1])


	def check_if_appropriate_distance_from_person(self, distance):
		if distance < self.allowed_dist_from_person:
			print "within correct distance"
			self.vel_msg.angular.z = 0
			self.vel_msg.linear.x = 0
			return True
		else:
			False


	def visualize_legs(self, data, angle_of_legs):
		wrapped_angles = data.ranges[355:360] + data.ranges + data.ranges[0:5] # add 5 to get the right value per angle
		person_points = []
		for angle in range(angle_of_legs - 5, angle_of_legs + 5):
			distance = wrapped_angles[angle + 5]
			point = self.calculate_point_position(angle, distance)
			person_points.append(point)
		self.marker.points = person_points
		self.marker_publisher.publish(self.marker)


	def calculate_point_position(self, angle, distance):
		fixed_frame_point_angle = self.angle + math.radians(angle) # angle of robot + angle of point
		x_position = self.position_x + math.cos(fixed_frame_point_angle) * distance
		y_position = self.position_y + math.sin(fixed_frame_point_angle) * distance
		return Vector3(x = x_position, y = y_position)


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
	node = person_follower()
	node.run()