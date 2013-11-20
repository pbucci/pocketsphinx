#!/usr/bin/env python


"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math
import intentmap

from std_msgs.msg import String

class voice_cmd_vel:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

		# Initialize message
		self.msg = std_msgs.msg.String()

        # publish to intent_pub, subscribe to speech output
        self.pub_ = rospy.Publisher('intent_pub', std_msgs.msg.String)
        rospy.Subscriber('recognizer/output', String, self.speechCb)

		# Get dummy intents for now
		self.intents = intentmap.intents

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.pub_.publish(self.msg)
            r.sleep()

    def speechCb(self, msg):
        rospy.loginfo(msg.data)

		# Deal with messages here

        self.pub_.publish(self.msg)

    def cleanup(self):
        # stop the robot!
		# from pirobot
		#		twist = Twist()
		#		self.pub_.publish(twist)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()
    except:
        pass