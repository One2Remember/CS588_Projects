#!/usr/bin/env python3

#==============================================================================
# File name : mp2.py 
# Description : MP2 for CS588 
# Usage : rosrun mp2 mp2.py 
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

# PID controller
from simple_pid.pid import PID



SHIFT_REVERSE_VALUE = 1
SHIFT_NEUTRAL_VALUE = 2
SHIFT_FORWARD_VALUE = 3

DISENGAGE_MESSAGE_VALUE = 0.0
BRAKE_MESSAGE_VALUE = 0.5
# ACCELERATE_MESSAGE_VALUE = 0.4

# prevent quick switching between forward/reverse
# will the lack of motion cause the pid controller to output a bigger acc value?
# is it better to artifically set the pid input equal to setpoint when similar?
# or do both ?
MOTION_MARGIN = 0.2

PID_SETPOINT = 10 # Area of bounding box # idk actual reasonable value yet
PID_SETPOINT_MARGIN = 1
ACC_LIMIT_LOW = -0.5
ACC_LIMIT_HIGH = 0.5

HUMAN_CLASS = 0
CAMERA_TOPIC = '/zed2/zed_node/rgb_raw/image_raw_color'


class FollowPedestrian():

    def __init__(self):
        self.rate = rospy.Rate(10)

        # Accelerate message init
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = True
        self.accel_cmd.clear = False
        self.accel_cmd.ignore = False
        self.accel_cmd.f64_cmd = DISENGAGE_MESSAGE_VALUE
        # Brake message init
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = True
        self.brake_cmd.clear = False
        self.brake_cmd.ignore = False
        self.brake_cmd.f64_cmd = DISENGAGE_MESSAGE_VALUE
        # Shift message init
        self.shift_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.shift_cmd = PacmodCmd()
        self.shift_cmd.enable = True # include ?
        self.shift_cmd.clear = False
        self.shift_cmd.ignore = False
        self.shift_cmd.ui16_cmd = SHIFT_NEUTRAL_VALUE

        # Load the YOLO model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True, device="cpu")
        self.human_detected = False
        self.bb_size = None

        # Subscribe to camera topic
        self.image_sub = rospy.Subscriber(CAMERA_TOPIC, Image, self.detect_human)

        # PID controller init
        kp, ki, kd = 1, 0.1, 0.05 # update ?
        self.pid = PID(kp, ki, kd, setpoint=PID_SETPOINT,output_limits = (ACC_LIMIT_LOW, ACC_LIMIT_HIGH))



    def run(self):
        while not rospy.is_shutdown():

            if self.human_detected:
                # get pid control from detector box size
                # add margin to prevent rapidly switching forward/reverse
                if abs(self.bb_size - PID_SETPOINT) < PID_SETPOINT_MARGIN:
                    control = self.pid(PID_SETPOINT)
                else:
                    control = self.pid(self.bb_size)

                control = self.pid(self.bb_size)
                print("PID controller output:", control)

                if control > MOTION_MARGIN: # > 0:
                    # switch to forward
                    self.shift_cmd.ui16_cmd = SHIFT_FORWARD_VALUE
                    self.shift_pub.publish(self.shift_cmd)
                    # stop breaking
                    self.brake_cmd.f64_cmd = DISENGAGE_MESSAGE_VALUE
                    self.brake_pub.publish(self.brake_cmd)
                    # engage accelerator
                    self.accel_cmd.f64_cmd = control
                    self.accel_pub.publish(self.accel_cmd)
                    print("Accelerating Forward")

                elif control < -MOTION_MARGIN: # < 0:
                    # switch to reverse
                    self.shift_cmd.ui16_cmd = SHIFT_REVERSE_VALUE
                    self.shift_pub.publish(self.shift_cmd)
                    # stop breaking
                    self.brake_cmd.f64_cmd = DISENGAGE_MESSAGE_VALUE
                    self.brake_pub.publish(self.brake_cmd)
                    # engage accelerator
                    self.accel_cmd.f64_cmd = abs(control) # I think
                    self.accel_pub.publish(self.accel_cmd)
                    print("Accelerating Backward")

                else:
                    # stop accelerating
                    self.accel_cmd.f64_cmd = DISENGAGE_MESSAGE_VALUE
                    self.accel_pub.publish(self.accel_cmd)
                    # engage brakes
                    self.brake_cmd.f64_cmd = BRAKE_MESSAGE_VALUE
                    self.brake_pub.publish(self.brake_cmd)
                    print("Braking, Human Distance Within Margin")
                    
            else:
                # stop accelerating
                self.accel_cmd.f64_cmd = DISENGAGE_MESSAGE_VALUE
                self.accel_pub.publish(self.accel_cmd)
                # engage brakes
                self.brake_cmd.f64_cmd = BRAKE_MESSAGE_VALUE
                self.brake_pub.publish(self.brake_cmd)
                print("Braking, No Human Detected")
                
            # pid controller also has internal sample_time=0.01, returns same output if called too soon
            self.rate.sleep()


    def detect_human(self, image):
        # convert image for model
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(image, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # get model results
        results = self.model(img, size=720)
        results.print()
        results.render()
        # convert to pandas df
        box_df = results.pandas().xyxy[0]

        # get result with humans detected
        people_df = box_df.loc[box_df['class'] == HUMAN_CLASS]
        # print num of people detected
        print(people_df.size)
        # return true if at least 1 person detected
        self.human_detected = people_df.size > 0

        # get bounding box size for one person detected
        # (could/should also do biggest/closest person)
        if self.human_detected:
            person = people_df.loc[0]
            self.bb_size = (person['xmax'] - person['xmin']) * (person['ymax'] - person['ymin'])
        else:
            self.bb_size = None


if __name__ == '__main__':
    rospy.init_node('follow_pedestrian', anonymous=True)
    node = FollowPedestrian()
    node.run()
