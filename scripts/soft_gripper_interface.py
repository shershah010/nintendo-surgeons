#!/usr/bin/env python
"""
Serial Interface for Soft Gripper
Author: Chris Correa
"""
import numpy as np
import serial
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

class SoftGripperInterface():
    def __init__(self):
        """
        Serial Interface for Soft Gripper
        """
        # General setup
        self.state_pub = rospy.Publisher('soft_gripper_state', SoftGripperState, queue_size=10)
        self.subscriber = rospy.Subscriber('soft_gripper_cmd', SoftGripperCmd, self.command_listener)
        rospy.on_shutdown(self.shutdown)
        rospy.sleep(1)
        # Arduino setup
        self.found_device = None
        # looks for re'/dev/ttyACM[0-9]'
        for device in os.listdir('/dev/'):
            if device.startswith('ttyACM'):
                self.found_device = device
        if self.found_device is None:
            rospy.logerr('Could not find arduino board')
            rospy.signal_shutdown('Could not find arduino board')
            sys.exit()
        self.ser = serial.Serial(os.path.join('/dev', self.found_device), 9600)
        
        # CV setup
        self.image_sub = rospy.Subscriber('camera/color/image_raw', Image, self.update_image)
        self.camera_sub = rospy.Subscriber('camera/color/camera_info', CameraInfo, self.update_intrinsic)
        self.bridge = CvBridge()
        blue_params = cv2.SimpleBlobDetector_Params()
        blue_params.filterByArea = True
        blue_params.minArea = 200
        #blue_params.filterByConvexity = False
        #blue_params.filterByCircularity = True
        #blue_params.minCircularity = .6
        # blue_params.filterByColor = True
        # blue_params.blobColor = 
        red_params = cv2.SimpleBlobDetector_Params()
        red_params.filterByArea = True
        red_params.minArea = 30
        self.blue_detector = cv2.SimpleBlobDetector_create(blue_params)
        self.red_detector = cv2.SimpleBlobDetector_create(red_params)
        # self.H = np.eye(3)
        self.K = np.eye(3)
        self.G = np.eye(4)
        self.im = None
        self.base_pos = np.array([112,-35, -40])
        self.tip_pos = np.array([0,0,0])

    def command_listener(self, msg):
        """update_image
        Callback fn for gripper pump commands

        Parameters
        ----------
        msg : :obj:`lab4_pkg.SoftGripperCmd`
            gripper command message containing pump values
        """
        self.send_cmd(msg.left, msg.right)

    def send_cmd(self, left, right):
        """
        Sends gripper commands to soft robot.  Commands will be clipped to [0,250]

        Parameters
        ----------
        left : int
            pump command for left finger
        right : int
            pump command for right gripper
        """
        string = str(int(left)) + ',' + str(int(right)) + "\n"
        self.ser.write(string.encode())

    def update_intrinsic(self, camera_info_msg):
        self.K = np.reshape(np.array(camera_info_msg.K), (3, 3))
        # print(self.K)

    def update_image(self, img_msg):
        """
        listens and updates the current image

        Parameters
        ----------
        img_msg : sensor_msgs/Image
        """
        self.im = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

    def update_homography(self):
        """
        Every so often, make sure the homography matrix correctly maps finger points from pixels
        to mm's
        """
        if self.im is None:
            return
        hsv = cv2.cvtColor(self.im, cv2.COLOR_BGR2HSV)
        a = time()
        mask = red_mask(hsv)
        kernel = np.ones((15, 15),np.uint8)
        erosion = cv2.erode(mask, kernel, iterations = 1)
        dilation = cv2.dilate(erosion, kernel, iterations = 1)
        keypoints = self.red_detector.detect(255-dilation)
        tmp_box = np.array([kp.pt for kp in keypoints])
        if tmp_box.shape != (4,2):
            print('Only see {} board points'.format(tmp_box.shape[0]))
            # plt.imshow(dilation, cmap='gray')
            # plt.show()
            # im_with_keypoints = cv2.drawKeypoints(self.im, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            # plt.imshow(cv2.cvtColor(im_with_keypoints, cv2.COLOR_BGR2RGB))
            # plt.show()
            return
        #np.sum gives an array of each x + y and argmin of s gives the smallest value of x + y 
        #argmax of s gives largest value of x + y 
        #np.diff gives an array of each y - x and argmin of d gives the smallest value of y - x
        #argmax of d gives the largest value of y - x
        s = np.sum(tmp_box, axis=1)
        d = np.diff(tmp_box, axis=1)
        box = np.array([
            tmp_box[np.argmin(s)],
            tmp_box[np.argmin(d)],
            tmp_box[np.argmax(d)],
            tmp_box[np.argmax(s)]
        ])
        #creates an array of 4 distinct points

        H, mask = cv2.findHomography(PROJECTED_BOX, box, cv2.RANSAC, 5.0)
        RT = np.matmul(np.linalg.inv(self.K), H)
        # norm = np.sqrt(np.linalg.norm(RT[:,0])*np.linalg.norm(RT[:,1]))
        norm = np.linalg.norm(RT[:,0])
        RT = 1*RT/norm
        c1 = RT[:,0]
        c2 = RT[:,1]
        c3 = np.cross(c1,c2)
        T = RT[:,2]
        R = np.vstack((c1,c2,c3)).T
        W,U,Vt = cv2.SVDecomp(R)
        R = np.matmul(U,Vt)
        T = (T[np.newaxis]).T
        self.G = np.block([[R, T], [0, 0, 0, 1]])
        # print(self.G)
        #else:
        #    return    
        
        # Homography debugging/test code (when doing findHomography(box, PROJECTED_BOX))

        # print(cv2.convertPointsToHomogeneous(box.T))
        # homog_points = cv2.convertPointsToHomogeneous(box).T
        # homog_points = homog_points[:, 0]
        # print(homog_points)
        # projected_homog = H.dot(homog_points)
        # print(projected_homog)
        # print(cv2.convertPointsFromHomogeneous(projected_homog.T))

        # creates a box from the four distinct points --> creates a plane
        # print 'homog time', time() - a
        # height, width, _ = self.im.shape
        # new_image = np.zeros(self.im.shape)
        # for y in range(height):
        #     for x in range(width):
        #         count = 0
        #         p_prime = np.array([x, y, 1])
        #         p = self.H.dot(p_prime)
        #         p = (p / p[2]).astype(int)
     
        #         if p[0] >= 0 and p[0] < self.im.shape[1] and \
        #            p[1] >= 0 and p[1] < self.im.shape[0]:
        #             new_image[y,x] = self.im[p[1], p[0]]
        # plt.imshow(new_image)
        # plt.show()

    def project_finger(self, finger_pt):
        """Takes a finger point in image coordinates and maps it to cm

        Parameters
        ----------
        finger_pt : 2x1 :obj:`numpy.ndarray`

        Returns
        -------pt_array = [k)
p.pt for kp in keypoints]
        # print(pt_array)
        3x1 :obj:numpy.ndarray`
            projected point in the square 
        """
        H = np.eye(3)
        p = H.dot(np.append(finger_pt, [1]))
        return p / p[2]

    def compute_finger_pos(self, errors):
        """
        Takes current image and computes the finger position based on the two blue circles.
        Stores them in self.base_pos and self.tip_pos
        """


        if self.im is None:
            return np.array([0,0,1]), np.array([0,0,1])
        hsv = cv2.cvtColor(self.im, cv2.COLOR_BGR2HSV)
        mask = blue_mask(hsv)
        errors[0] += 1
        kernel = np.ones((10,10),np.uint8)
        dilation = cv2.dilate(mask, kernel, iterations = 1)
        keypoints = self.blue_detector.detect(255-dilation)
        # pt_array = [kp.pt for kp in keypoints]
        # print(pt_array)
        # finger_pts = np.array([self.project_finger(kp.pt) for kp in keypoints])
        finger_pts = np.array([kp.pt for kp in keypoints])
        # print(finger_pts)
        # if finger_pts.shape == (3, 2):
            # finger_pts = np.delete(finger_pts, np.argmax(finger_pts[:, 1]), 0)

        # Error display code
        if finger_pts.shape[0] < 2:
        #     error_im = self.im
        #     plt.imshow(dilation, cmap='gray')
        #     plt.show()
        #     im_with_keypoints = cv2.drawKeypoints(error_im, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        #     for ky in keypoints:
        #         im_with_keypoints = cv2.circle(im_with_keypoints, (int(ky.pt[0]), int(ky.pt[1])), 1, (0, 0, 255), -1)
        #     plt.imshow(cv2.cvtColor(im_with_keypoints, cv2.COLOR_BGR2RGB))
        #     plt.show()
            print 'Only see {} finger points'.format(finger_pts.shape[0])
            errors[1] += 1
            print(float(errors[1])/errors[0])
            return
        # self.base_pos = finger_pts[np.argmin(finger_pts[:,1])]
        # finger_pts = np.delete(finger_pts, np.argmin(finger_pts[:, 1]), 0)
        # center = np.array([[320, 240] for _ in range(finger_pts.shape[0])])
        # dists = np.linalg.norm(finger_pts - center, axis=1)
        # print(dists)
        # tip_pixel = finger_pts[np.argmin(dists)]
        # print(tip_pixel)
        real_pos = np.apply_along_axis(self.find_pos_blue, 1, finger_pts)
        dists = np.linalg.norm(real_pos - np.append(self.base_pos, [1]), axis = 1)
        self.base_pos = real_pos[np.argmin(dists)][:3]
        real_pos = np.delete(real_pos, np.argmin(dists), 0)
        dists = np.delete(dists, np.argmin(dists), 0)
        self.tip_pos = real_pos[np.argmin(dists)][:3]
        print('Base Pos: ' + str(self.base_pos))
        print('Tip Pos: ' + str(self.tip_pos))

    def find_pos_blue(self, blue_dot):
        tip_pixel = np.append(blue_dot, 1)
        n = np.array([0, 0, self.base_pos[2], 0])
        n_camera = self.G.dot(n)
        ray = np.linalg.inv(self.K).dot(tip_pixel)
        base_camera = self.G.dot(np.append(self.base_pos, [1]))
        lambduh = (base_camera[:3].dot(n_camera[:3]))/(ray.dot(n_camera[:3]))
        tip_camera = lambduh * ray
        return np.linalg.inv(self.G).dot(np.append(tip_camera, [1]))

    def run(self):
        """
        Listens for states across serial and publishes to ROS
        """
        print 'Running....'
        i = 0
        errors = [0, 0]
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            read_serial = self.ser.readline()
            try:
                if i % 20 == 0:
                    self.update_homography()
                if (len(read_serial.split(',')) == 7):
                    time, left_pwm, right_pwm, left_pressure, right_pressure, left_flex, right_flex = read_serial.split(',')
                self.compute_finger_pos(errors)
                state = SoftGripperState(
                    float(time),
                    float(left_pwm), float(right_pwm),
                    float(left_pressure), float(right_pressure), 
                    float(left_flex), float(right_flex),
                    Vector3(self.base_pos[0], self.base_pos[1], self.base_pos[2]), 
                    Vector3(self.tip_pos[0], self.tip_pos[1], self.tip_pos[2])
                )
                self.state_pub.publish(state)
                i += 1
            except:
                print 'Tried to parse:', read_serial
                traceback.print_exc(file=sys.stdout)
            rate.sleep()
        # Stop
        self.shutdown()
        print 'Stopping :('

    def shutdown(self):
        """
        If you connected to an arduino, stop it when exiting
        """
        if self.found_device:
            self.send_cmd(0,0)

if __name__ == '__main__':
    rospy.init_node('soft_gripper_interface', anonymous=True)
    sgi = SoftGripperInterface()
    sgi.run()
