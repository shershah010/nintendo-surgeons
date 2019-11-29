#!/usr/bin/env python
"""
Script to record data on the soft finger
Author: Chris Correa
"""
import numpy as np
import pandas as pd
import csv
import os

import rospy
from lab4_pkg.msg import SoftGripperState, SoftGripperCmd
import datetime

d = datetime.datetime.today()
test_pwm = 200

class DataRecorder():
    def __init__(self, run_name):
        """
        Records data for the soft finger

        Parameters
        ----------
        run_name : string
            name of file to save to.  should include .csv at the end
        """
        self.cmd_pub = rospy.Publisher('soft_gripper_cmd', SoftGripperCmd, queue_size=10)
        rospy.sleep(1)
        rospy.Subscriber('soft_gripper_state', SoftGripperState, self.state_listener)
        rospy.on_shutdown(self.shutdown)
        self.test_pwm = 200
        self.states = []
        self.rate = rospy.Rate(100)
        self.start_time = None
        self.run_name = run_name

    def state_listener(self, msg):
        """
        Records states from the 'soft_gripper_state' topic

        Parameters
        ----------
        msg : :obj:`lab4_pkg.SoftGripperState`
        """
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

    def flush(self):
        """
        writes states to file
        """
        df = pd.DataFrame(
            np.array(self.states), 
            columns=[
                'time', 
                'left_pwm', 'right_pwm', 
                'left_pressure', 'right_pressure', 
                'left_flex', 'right_flex', 
                'base_pos_x', 'base_pos_y', 
                'tip_pos_x', 'tip_pos_y'
            ]
        )
        filename = os.path.join(os.path.dirname(os.path.realpath('__file__')), 'data', self.run_name)
        df.to_csv(filename)

    def record_data(self):
        """
        Script to command soft finger.  You can send commands to both fingers, but only the right is attached.
        """
        self.cmd_pub.publish(SoftGripperCmd(0,0))
        rospy.sleep(2)
        self.cmd_pub.publish(SoftGripperCmd(test_pwm,0))
        rospy.sleep(10)
        self.cmd_pub.publish(SoftGripperCmd(0,0))
        rospy.sleep(8)
        self.shutdown()

    def shutdown(self):
        """
        Stops the finger and flushes whenever you exit
        """
        self.cmd_pub.publish(SoftGripperCmd(0,0))
        self.flush()

if __name__ == '__main__':
    rospy.init_node('data_recorder')
    for pwm in range(20, 201, 20):
        for i in range(1):
            d = datetime.datetime.today()
            test_pwm = pwm
            dr = DataRecorder('test_data_pwm' + str(test_pwm) + '_' + d.strftime('%d_%m_%Y_%H_%M_%S') + '.csv')
            dr.record_data()
            dr.flush()
