#!/usr/bin/env python

"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import roslib; roslib.load_manifest('turtlebot_vicos')
import rospy
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class voice_cmd_vel:

    def __init__(self, prefix):
        rospy.on_shutdown(self.cleanup)
        self.speed = 0.2
        self.msg = Twist()
        self.prefix = prefix

        # publish to cmd_vel, subscribe to speech output
        self.pub_ = rospy.Publisher('cmd_vel', Twist)
        rospy.Subscriber('commands', String, self.speechCb)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.pub_.publish(self.msg)
            r.sleep()
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)

        if not msg.data.startswith(self.prefix):
            return

        command = msg.data[len(self.prefix):].strip()

        if command == "full speed":
            if self.speed == 0.2:
                self.msg.linear.x = self.msg.linear.x*2
                self.msg.angular.z = self.msg.angular.z*2
                self.speed = 0.4
        if command == "half speed":
            if self.speed == 0.4:
                self.msg.linear.x = self.msg.linear.x/2
                self.msg.angular.z = self.msg.angular.z/2
                self.speed = 0.2

        if command == "move forward":    
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
        elif command == "move left":
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z += 0.05
            else:        
                self.msg.angular.z = self.speed*2
        elif command == "move right":    
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z -= 0.05
            else:        
                self.msg.angular.z = -self.speed*2
        elif command == "move backward":
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0
        elif command.find("stop") > -1 or command.find("halt") > -1:          
            self.msg = Twist()
        
        self.pub_.publish(self.msg)

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.pub_.publish(twist)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    prefix = rospy.get_param('~prefix', '')

    try:
        voice_cmd_vel(prefix)
    except:
        pass

