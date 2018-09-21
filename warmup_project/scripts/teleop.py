#!/usr/bin/env python
""" This script moves a robot with key commands """
import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist


class teleoperate_robot(object):

	def __init__(self):
		""" initializes teleop object """
		# initialize rospy
		rospy.init_node('bumpty_dumpty')
		self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
		self.rate = rospy.Rate(10)

		# initialize speed
		self.vel_msg = Twist()
		self.set_still_state()

		# initalize keyboard settings
		self.settings = termios.tcgetattr(sys.stdin)

	def set_still_state(self):
		""" stops robot from moving linearly or angularly """
		self.vel_msg.linear.x = 0
		self.vel_msg.angular.z = 0

	def run(self):
		""" starts the teleop functionality """
		key = None
		while key != '\x03' and not rospy.is_shutdown():
			key = self.get_key()
			self.process_key(key)
			self.publisher.publish(self.vel_msg) # tell robot where to move
			self.rate.sleep()

	def get_key(self):
		""" gets the key that was pressed """
		tty.setraw(sys.stdin.fileno())
		select.select([sys.stdin], [], [], 0)
		key = sys.stdin.read(1)
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return key

	def process_key(self, key):
		""" turns a keypress into movement """
		if key == 'q': # forward-left
			self.vel_msg.linear.x = .5
			self.vel_msg.angular.z = 1
		elif key == 'w': # forward
			self.set_still_state()
			self.vel_msg.linear.x = 1
		elif key == 'e': # forward-right
			self.vel_msg.linear.x = .5
			self.vel_msg.angular.z = -1
		elif key == 'a': # left (turn in place)
			self.set_still_state()
			self.vel_msg.angular.z = 1
		elif key == 's': # stop moving
			self.set_still_state()
		elif key == 'd': # right (turn in place)
			self.set_still_state()
			self.vel_msg.angular.z = -1
		elif key == 'z': # backwards-left
			self.vel_msg.linear.x = -.5
			self.vel_msg.angular.z = 1
		elif key == 'x': # backwards
			self.set_still_state()
			self.vel_msg.linear.x = -1
		elif key == 'c': # backwards-right
			self.vel_msg.linear.x = -.5
			self.vel_msg.angular.z = -1



if __name__ == '__main__':
	node = teleoperate_robot()
	node.run()