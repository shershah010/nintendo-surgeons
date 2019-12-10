#!/usr/bin/env python
"""
Camera Test/Experiment Code
Author: Chris Correa
"""
import numpy as np
import sys
from cv_bridge import CvBridge
import cv2
import matplotlib.pyplot as plt
from time import time
import traceback

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image, CameraInfo

from lab4_pkg.msg import SoftGripperState, SoftGripperCmd
class CameraTest:
    
    def __init__(self):
    
        self._base_pos =[]
        self._tip_pos = []
        self.state_pub = rospy.Publisher('soft_gripper_cmd', SoftGripperState, queue_size=10)
        self.subscriber = rospy.Subscriber('soft_gripper_state', SoftGripperCmd, self.state_listener)

    def state_listener(self, msg):
        if self.start_time is None:
            self.start_time = msg.time
        self.states.append([
            msg.time - self.start_time, 
            msg.left_pwm, msg.right_pwm, 
            msg.left_pressure, msg.right_pressure, 
            msg.left_flex, msg.right_flex, 
            msg.base_pos.x, msg.base_pos.y, 
            msg.tip_pos.x, msg.tip_pos.y
        ])
