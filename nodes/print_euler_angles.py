#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 25 17:50:58 2019

@author: ckielasjensen
"""

from geometry_msgs.msg import PoseStamped
import rospy
import tf


topic = '/vrpn_client_node/Turtlebot3_5/pose'


def callback(data):

    quat = (data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quat, axes='rzyx')

    yaw = euler[0]
    pitch = euler[1]
    roll = euler[2]

    print('Roll: {:4.2f}, Pitch: {:4.2f}, Yaw: {:4.2f}'.format(
        roll, pitch, yaw))


if __name__ == '__main__':
    rospy.init_node('roll_pitch_yaw_printer', anonymous=True)

    rospy.Subscriber(topic, PoseStamped, callback)

    rospy.spin()