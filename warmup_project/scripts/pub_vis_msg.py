#!/usr/bin/env python
""" This script explores publishing ROS messages in ROS using Python """
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
import rospy


class TestRviz(object):
	def __init__(self):
		rospy.init_node('test_message')    # initialize ourselves with roscore
		self.publisher = rospy.Publisher('/my_marker', Marker, queue_size=10)
		self.my_marker = Marker()
		self.my_marker.type = Marker.SPHERE
		self.my_marker.pose.position.x = 1
		self.my_marker.pose.position.y = 2
		self.my_marker.scale.x = 1
		self.my_marker.scale.y = 1
		self.my_marker.scale.z = 1
		self.my_marker.color.a = 1 # makes it visible
		self.my_marker.color.r = 1 # should make it red?
		self.my_marker.header.frame_id = "odom"
		self.rate = rospy.Rate(10)

	def run(self):		
		while not rospy.is_shutdown():
		    self.publisher.publish(self.my_marker)
		    self.rate.sleep()


if __name__ == '__main__':
	node = TestRviz()
	node.run()
