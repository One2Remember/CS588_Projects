#!/usr/bin/env python3

#==============================================================================
# File name          : mp1.py                                                                 
# Description        : MP1 for CS588                                                                                                                        
# Usage              : rosrun mp1 mp1.py                                                                                                                           
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
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd

BREAK_MESSAGE_VALUE = 0.5
ACCELERATE_MESSAGE_VALUE = 0.5
DISENGAGE_MESSAGE_VALUE = 0.0

class BreakForPedestrian():

	def __init__(self):
        self.rate = rospy.Rate(10)
        
        # Accelerate message init
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = True
        self.accel_cmd.clear  = False
        self.accel_cmd.ignore = False
        self.accel_cmd.f64_cmd = DISENGAGE_MESSAGE_VALUE
    
		# Brake message init
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = True
        self.brake_cmd.clear  = False
        self.brake_cmd.ignore = False
        self.brake_cmd.f64_cmd = DISENGAGE_MESSAGE_VALUE


	def run(self):
		while not rospy.is_shutdown():
            #todo: determine if human is detected
            humanDetected = True
        
            if humanDetected:
                self.brake_cmd.f64_cmd = BREAK_MESSAGE_VALUE
                self.brake_pub.publish(self.brake_cmd)
                
                self.accel_cmd.f64_cmd = DISENGAGE_MESSAGE_VALUE
                self.accel_pub.publish(self.accel_cmd)
                
                print("Braking")
            else:
                self.brake_cmd.f64_cmd = DISENGAGE_MESSAGE_VALUE
                self.brake_pub.publish(self.brake_cmd)

                self.accel_cmd.f64_cmd = ACCELERATE_MESSAGE_VALUE
                self.accel_pub.publish(self.accel_cmd)
                
                print("Accelerating")
                
            self.rate.sleep()


if __name__ == '__main__':
	rospy.init_node('brake_for_pedestrian', anonymous=True)
	node = BreakForPedestrian()
	node.run()
