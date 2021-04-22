#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 26 14:38:23 2019

@author: ckielasjensen
"""

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import numpy as np
import rospy

PERIOD = 10 # ms
FREQ = 100 # Hz
R = 1.25 # Radius in meters
OMEGA = 1 #0.14
GAMMA_TIME = False


def gamma_cb(time, gamma):
    gamma.data = time.data


if __name__ == '__main__':
    rospy.init_node('sinusoid_generator', anonymous=True)
    rate = rospy.Rate(FREQ)
    pub0 = rospy.Publisher('/AR_0/cmd_pose', PoseStamped, queue_size=10)
    pub1 = rospy.Publisher('/AR_1/cmd_pose', PoseStamped, queue_size=10)
    pub2 = rospy.Publisher('/AR_2/cmd_pose', PoseStamped, queue_size=10)

    ps0 = PoseStamped()
    ps1 = PoseStamped()
    ps2 = PoseStamped()

    ps0.header.frame_id = 'world'
    ps1.header.frame_id = 'world'
    ps2.header.frame_id = 'world'

    if GAMMA_TIME:
        gamma = Float64()
        rospy.Subscriber('/gamma1', Float64, lambda x: gamma_cb(x, gamma))

    initTime = rospy.Time.now()
    while not rospy.is_shutdown():

        if not GAMMA_TIME:
            t = (rospy.Time.now()-initTime).to_sec()
        else:
            t = gamma.data

        x0 = R*np.cos(OMEGA*t)
        y0 = R*np.sin(OMEGA*t)
        x1 = R*np.cos(OMEGA*t+2*np.pi/3)
        y1 = R*np.sin(OMEGA*t+2*np.pi/3)
        x2 = R*np.cos(OMEGA*t+4*np.pi/3)
        y2 = R*np.sin(OMEGA*t+4*np.pi/3)

        ps0.pose.position.x = x0
        ps0.pose.position.y = y0
        ps1.pose.position.x = x1
        ps1.pose.position.y = y1
        ps2.pose.position.x = x2
        ps2.pose.position.y = y2

        ps0.header.stamp = rospy.Time.now()
        ps1.header.stamp = rospy.Time.now()
        ps2.header.stamp = rospy.Time.now()

        pub0.publish(ps0)
        pub1.publish(ps1)
        pub2.publish(ps2)

        rate.sleep()