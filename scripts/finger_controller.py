#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import os

import rospy
from lab4_pkg.msg import SoftGripperState, SoftGripperCmd
import datetime

import sys
import itertools

from moveit_msgs.msg import RobotTrajectory



class Controller(object):
    def __init__(self, q_d, t_d, Kp, Ki, Kd, Kw, limb):

        self.time_data = []
        self.angle_data = []
        self.error_data = []
        self.u_data = []
        self._t_i = None
        self._q_i = 0
        self._q_d = q_d
        self._t_d = t_d
        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._Kw = Kw

        #self._LastError = np.zeros(len(Kd))
        self._LastError = 0
        self._LastTime = 0
        self._IntError = 0
        self._a_ring_buff = [0 for _ in range(5)]
        self._d_ring_buff = [0 for _ in range(3)]
        self._curIndex = 0
        self._maxIndex = 0

        self._limb = limb

        self.cmd_pub = rospy.Publisher('soft_gripper_cmd', SoftGripperCmd, queue_size=10)
        rospy.sleep(1)
        rospy.Subscriber('soft_gripper_state', SoftGripperState, self.state_listener)
        #self._q_eq = self.q_desired_line()
        rospy.on_shutdown(self.shutdown)

    def q_desired_line(self):
        q_vel = (self._q_d - self._q_i) / (self._t_d)
        def q_eq(t):
            if t > self._t_d:
                line_eq = self._q_d
            else:
                line_eq = q_vel*t + self._q_i
            return line_eq
        return q_eq

    def sin_eq(self, t):
        sin = self._q_d + self._q_i + self._q_d*np.sin((2*np.pi / (4 * self._t_d))*(t - self._t_d))
        return sin


    def C(self, q, q_vel):
        #coreolis func
        return 1

    def G(self, q):
        L_i = 80
        g = L_i*(np.sin(q) / (2*q))
        return g
    
    def u_dynamics(self, alpha, gamma, tau_vel, damping, K):
        alph, gam, t_dot, D, k = alpha, gamma, tau_vel, damping, K
        q_eq = self.q_desired_line()
        def u_eq(t):
            q = q_eq(t)
            q_vel = (q / t) - self._q_i
            c = self.C(q, q_vel)
            g = self.G(q)
            u = (1 / alph)*(c + D)*q_vel + (1 / alph)*g + (1 / alph)*k*q + (2 / alph)*gam*t_dot
            return u
        return u_eq

    def u_ff_eq(self, t):
        #u_ff*f(t) = F(t)= ma(t)
        m = 0.052
        sin_func = -0.025*self._q_d*np.sin((2*np.pi / (4 * self._t_d)) *(t - self._t_d))
        uff_func = m*sin_func
        return uff_func

    def flex2angle(self, flex):
        # a = -1.05591989
        # b = 9.33421143e-4
        # c = 297.1056114257644
        a = -.986365423
        b = .00088301426
        c = 274.448006079
        theta = a*flex + b*flex*flex + c #+ d
        return theta

    def state_listener(self, msg):
        """
        Records states from the 'soft_gripper_state' topic
        Parameters
        ----------
        msg : :obj:`lab4_pkg.SoftGripperState`
        """
        if self._t_i is None:
            self._t_i = msg.time
            self._q_i = self.flex2angle(msg.left_flex)

        else: 
            self._a_ring_buff[1:] = self._a_ring_buff[:(len(self._a_ring_buff) - 1)]
            self._a_ring_buff[0] = self.flex2angle(msg.left_flex)
            self.state = {
            'time':msg.time - self._t_i, 
            # msg.left_pwm, msg.right_pwm, 
            # msg.left_pressure,
            # msg.right_pressure, 
            # 'angle':self.flex2angle(msg.left_flex)
            'angle':np.mean(self._a_ring_buff)
            # msg.right_flex, 
            # msg.base_pos.x, msg.base_pos.y, 
            # msg.tip_pos.x, msg.tip_pos.y
            }
            print("Time: " + str(self.state['time']))
            self.time_data.append(self.state['time'])
            self.angle_data.append(self.state['angle'])
            self.step_control()

    def shutdown(self):
        """
        Code to run on shutdown. This is good practice for safety
        """
        rospy.loginfo("Stopping Controller")

        # Set velocities to zero
        self.cmd_pub.publish(SoftGripperCmd(0,0))
        self.cmd_pub.publish(SoftGripperCmd(0,0))
        self.cmd_pub.publish(SoftGripperCmd(0,0))
        self.cmd_pub.publish(SoftGripperCmd(0,0))
        self.cmd_pub.publish(SoftGripperCmd(0,0))

        self.cmd_pub.publish(SoftGripperCmd(0,0))
        self.cmd_pub.publish(SoftGripperCmd(0,0))
        self.cmd_pub.publish(SoftGripperCmd(0,0))



    def step_control(self):
        """
        Return the control input given the current controller state at time t

        Inputs:
        t: time from start in seconds

        Output:
        u: 7x' ndarray of velocity commands
        
        """


        # # Feed Forward Term
        # u_ff = target_velocity

        # # Error Term
        #error = self._q_eq(self.state['time']) - self.state['angle']
        error = self.sin_eq(self.state['time']) - self.state['angle']
        self.error_data.append(error)

        dt = self.state['time'] - self._LastTime

        # Integral Term
        self._IntError = self._Kw * self._IntError + error * dt
        # # Derivative Term
        # # We implement a moving average filter to smooth the derivative
        self._d_ring_buff[1:] = self._d_ring_buff[:(len(self._d_ring_buff) - 1)]
        self._d_ring_buff[0] = (error - self._LastError) / (1.0 * dt)
        ed = np.mean(self._d_ring_buff)

        # # Save terms for the next run

        # Note, you should load the Kp, Ki, Kd, and Kw constants with
        # self._Kp
        # and so on. This is better practice than hard-coding
        u_ff = 3.5 * self.sin_eq(self.state['time'])#self.u_ff_eq(self.state['time'])
        u = u_ff + self._Kp * error + self._Kd * ed + self._Ki * self._IntError
        u = u if u >= 40 else 40
        self.u_data.append(u)
        self.cmd_pub.publish(SoftGripperCmd(u,0))
        print("Error: " + str(error))
        print("Derivative Error: " + str(ed))
        print("Integral Error: " + str(self._IntError))
        self._LastError = error
        self._LastTime = self.state['time']



if __name__ == '__main__':
    rospy.init_node('controller')
    user_angle = input('input the desired angle: ')
    user_time = input('input the desired time: ')
    # c = Controller(user_angle, user_time, 1, .1, 20, .75, 'left')
    # c = Controller(user_angle, user_time, 0.5, 0.5, 25, .75, 'left')
    c = Controller(user_angle, user_time, 2, 10, .6, .9, 'left')
    rospy.on_shutdown(c.shutdown)
    while not rospy.is_shutdown():
        continue
    c.shutdown()
    time_array = np.array(c.time_data)
    angle_array = np.array(c.angle_data)
    plt.plot(time_array, angle_array, label='actual')
    trajectory = np.apply_along_axis(c.sin_eq, 0, time_array)
    plt.plot(time_array, trajectory, label='trajectory')
    plt.plot(time_array, np.array(c.error_data), label='error')
    plt.plot(time_array, np.array(c.u_data), label='pwm')
    plt.show()
    c.shutdown()
