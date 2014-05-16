#!/usr/bin/env python
import roslib; roslib.load_manifest('turtlebot_vicos')
import rospy
import math
import cmd
import sys

from std_msgs.msg import String

class RobotChat(cmd.Cmd):

	def __init__(self):
		cmd.Cmd.__init__(self)

		self.publisher = rospy.Publisher('output', String)
		self.prompt = ':-O '
		self.cmdloop('Write commands to publish them to the output topic. Type ? for help.')

	def default(self, line):
		message = String()
		message.data = line
		self.publisher.publish(message)

	def do_help(self, line):
		print 'All the non-special messages will be sent to the output topic'
		print 'You can use arrow keys to browse the history'
		print 'Type !quit to exit the chat terminal'

	def do_shell(self, line):
		if line == 'quit':
			sys.exit()


if __name__=="__main__":
	rospy.init_node('robot_chat')
	RobotChat()

