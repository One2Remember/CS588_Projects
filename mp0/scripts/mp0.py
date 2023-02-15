#!/usr/bin/env python3

#==============================================================================
# File name          : mp0.py                                                                 
# Description        : MP0 for CS588                                                                                                                        
# Usage              : rosrun mp0 mp0.py                                                                                                                           
#==============================================================================
from __future__ import print_function

#Python Headers
import math
import os
import time

# ROS Headers
import rospy

# GEM PACMod Headers
from std_msgs.msg import Header
from pacmod_msgs.msg import PacmodCmd

#left-left-left-right-right-right-left-left-left

#DOT REST
DOT  = rospy.Duration.from_sec(0.25)
#DASH REST
DASH = rospy.Duration.from_sec(0.5)
#OFF REST
OFF  = rospy.Duration.from_sec(0.25)
#VERY LONG REST
VLR  = rospy.Duration.from_sec(3.0)

class Node():

	def __init__(self):
		# left-left-left-right-right-right-left-left-left
		# 0: TURN_RIGHT
		# 1: TURN_NONE
		# 2: TURN_LEFT
		self.signal_list = [2,1,2,1,2,1,0,1,0,1,0,1,2,1,2,1,2,1]
        self.sleep_rates = [DOT,OFF,DOT,OFF,DOT,OFF,DASH,OFF,DASH,OFF,DASH,OFF,DOT,OFF,DOT,OFF,DOT,VLR]
		self.signal_list_pos = 0
		self.headlight_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size = 1)


	def run(self):
		while not rospy.is_shutdown():
			# call function to send headlight command
			self.sos_headlights()
            # sleep for appropriate time
			rospy.sleep(self.sleep_rates[self.signal_list_pos])
            # increment signal list position
			self.signal_list_pos += 1
            # reset if we go over
            if(self.signal_list_pos >= len(self.signal_list)):
                self.signal_list_pos = 0
	
	def sos_headlights(self):
        # Get headlight command from signal_list
        headlight_cmd = self.signal_list[self.signal_list_pos]

        # Create PacmodCmd message and populate with data
        msg = PacmodCmd()
        msg.ui16_cmd = headlight_cmd

        # Publish message to headlight_cmd topic publisher
        self.headlight_pub.publish(msg)


if __name__ == '__main__':
	rospy.init_node('sos_node', anonymous=True)
	node = Node()
	node.run()
