#!/usr/bin/env python
"""
Camera Test/Experiment Code
Author: Chris Correa
"""
import numpy as np
import os
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

blue_mask = lambda hsv: cv2.inRange(hsv, np.array([80,100,100]), np.array([130,255,255]))#cv2.inRange(hsv, np.array([110,50,50]), np.array([130,255,255]))
# This is really stupid.  why would red HSV not be all consecutive.....
red_mask =  lambda hsv: cv2.inRange(hsv, np.array([0,50,50]), np.array([10,255,255])) | cv2.inRange(hsv, np.array([170,50,50]), np.array([180,255,255]))

PROJECTED_BOX = np.array([[0,0],[235,0],[0,185],[235,185]]) # measured dimensions of 4 red points

#Record camera data
#Display a test procedure (allow for some custimization, time length, etc)
#Plot data over time

class CameraTest:
    
    def __init__(self):
    
        self._base_pos =[]
        self._tip_pos = []

