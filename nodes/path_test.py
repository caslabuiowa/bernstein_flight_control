#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 20 16:19:24 2019

@author: ckielasjensen
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


if __name__ == '__main__':

    rospy.init_node('path_test_node')

    path = Path()
    pose = PoseStamped()

    path.header.frame_id = 'world'
    pose.header.frame_id = 'world'

    pathPub = rospy.Publisher('/test/path', Path, queue_size=10)
    posePub = rospy.Publisher('/test/pose', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        curTime = rospy.Time.now()
        path.header.stamp = curTime
        pose.header.stamp = curTime

        pose.pose.position.x += 1
        pose.pose.position.y = pose.pose.position.y**2

        path.poses.append(pose)

        pathPub.publish(path)
        posePub.publish(pose)