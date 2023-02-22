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
import torch
import cv2
import numpy as np
from PIL import Image as im
import time

# ROS Headers
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# GEM PACMod Headers
from std_msgs.msg import Header
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd

BREAK_MESSAGE_VALUE = 0.5
ACCELERATE_MESSAGE_VALUE = 0.5
DISENGAGE_MESSAGE_VALUE = 0.0
HUMAN_CLASS = 0
CAMERA_TOPIC = '/'

class BreakForPedestrian():

    def __init__(self):
        self.rate = rospy.Rate(10)
        self.human_detected = False
        
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

        # Load the YOLO model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True, device=0)

        self.image_sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.detect_human)



    def run(self):
        while not rospy.is_shutdown():
        
            if self.human_detected:
                # stop accelerating
                self.accel_cmd.f64_cmd = DISENGAGE_MESSAGE_VALUE
                self.accel_pub.publish(self.accel_cmd)

                # engage brakes
                self.brake_cmd.f64_cmd = BREAK_MESSAGE_VALUE
                self.brake_pub.publish(self.brake_cmd)
                
                print("Braking")

            else:
                # stop breaking
                self.brake_cmd.f64_cmd = DISENGAGE_MESSAGE_VALUE
                self.brake_pub.publish(self.brake_cmd)

                # engage accelerator
                self.accel_cmd.f64_cmd = ACCELERATE_MESSAGE_VALUE
                self.accel_pub.publish(self.accel_cmd)
                
                print("Accelerating")
                
            self.rate.sleep()
            rospy.spin()
    
    
    def detect_human(self, image):
        # convert image for model
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(image, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # get model results
        results = self.model(img)
        results.print()
        results.render()
        
        # convert to pandas df
        box_df = results.pandas().xyxy[0]
        # get result with humans detected
        people_df = box_df.loc[box_df['class'] == HUMAN_CLASS]
        # print # of people detected
        print(people_df.size)
        # return true if at least 1 person detected
        self.human_detected = people_df.size > 0
    


if __name__ == '__main__':
    rospy.init_node('brake_for_pedestrian', anonymous=True)
    node = BreakForPedestrian()
    node.run()
