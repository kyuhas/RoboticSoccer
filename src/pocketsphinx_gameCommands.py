#!/usr/bin/env python

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math
import time

from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Header, String

class voice_game_commands():
	def __init__(self):
		# create publisher
		self.pub = rospy.Publisher('/gameCommands', String, queue_size=10)
		
		# subscribe to speech output and amcl pose
		rospy.Subscriber('recognizer/output', String, self.speechCb)

		time.sleep(1) #give publisher time to get set up
		rospy.spin()

	def determineCmd(self, cmdStr):
		if "start" in cmdStr:
			self.pub.publish("start")
		elif "pause" in cmdStr:
			self.pub.publish("pause")
		elif "stop" in cmdStr:
			self.pub.publish("stop")
		elif "field" in cmdStr:
			self.pub.publish("field")
		
	def speechCb(self, msg):
		self.determineCmd(msg.data)

if __name__=="__main__":
    rospy.init_node('voice_game_commands')
    try:
        voice_game_commands()
    except:
        pass

