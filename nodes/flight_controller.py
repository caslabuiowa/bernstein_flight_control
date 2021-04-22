#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  5 11:58:19 2019

@author: ckielasjensen
"""

from geometry_msgs.msg import PoseStamped, PointStamped, Twist, Pose
import rospy
from simple_pid import PID
import tf
import numpy as np
import time

TESTING = False
SAMPLE_TIME = None   # 0.02   # None   # 1.0/100
WAIT_DURATION = 1.0  # 10*SAMPLE_TIME


def readCmd(data, cmd_pose):
    cmd_pose[0] = data
    cmd_pose[0].header.frame_id = 'world'


def callback(data, pub, pidX, pidY, pidW, cmd_pose, listener, debugPub):
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z

    quat = (data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quat)

    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    try:
        ros_time_now = rospy.Time.now()

#        ps = PointStamped()
#        ps.header = data.header
#        ps.point = data.pose.position
#        ps.header.stamp = ros_time_now

        ps = PointStamped()
        ps.header = cmd_pose[0].header
        ps.point = cmd_pose[0].pose.position
        ps.header.stamp = ros_time_now

        listener.waitForTransform('world', 'Mambo_0', ros_time_now,
                                  rospy.Duration(WAIT_DURATION))
        robot_frame_cmd_pos = listener.transformPoint('Mambo_0', ps)

        

        
        x_cmd = pidX(-robot_frame_cmd_pos.point.y)
        x_cmd_raw = 10*robot_frame_cmd_pos.point.y
        y_cmd = pidY(-robot_frame_cmd_pos.point.x)
        y_cmd_raw = 10*robot_frame_cmd_pos.point.x
        w_cmd = 0 #pidW(yaw)
        
#        debugData = Pose()
#        debugData.position.x = -robot_frame_cmd_pos.point.y
#        debugData.position.y = -robot_frame_cmd_pos.point.x
        debugData = Twist()
        debugData.linear.x = x_cmd_raw
        debugData.linear.y = y_cmd_raw
        debugPub.publish(debugData)

#        xErr = cmd_pose[0].pose.position.x - robot_frame_cmd_pos.point.x
#        yErr = cmd_pose[0].pose.position.y - robot_frame_cmd_pos.point.y
#        wErr = cmd_pose[0].pose.orientation.w - yaw
#
#        x_cmd = pidX(xErr)
#        y_cmd = pidY(yErr)
#        w_cmd = pidW(wErr)

#        pidX.setpoint = cmd_pose[0].pose.position.x
#        pidY.setpoint = cmd_pose[0].pose.position.y
#        pidW.setpoint = cmd_pose[0].pose.orientation.w
#
#        x_cmd = pidX(x)
#        y_cmd = pidY(y)
#        w_cmd = pidW(yaw)

        cmd_vel = Twist()
        cmd_vel.linear.x = 0 #x_cmd
        cmd_vel.linear.y = 20*np.sin(1.2*time.time()) #y_cmd
        cmd_vel.angular.z = 0 #w_cmd

        if TESTING:
#            msg = ('\nRoll: {:.3f}, Pitch: {:.3f}, Yaw: {:.3f}'
#                   '\n   X: {:.3f},     Y: {:.3f},   Z: {:.3f}').format(
#                   roll, pitch, yaw, x, y, z)
#            msg = ('\nMamboFrame X: {:.3f} Y: {:.3f}').format(
#                    robot_frame_cmd_pos.point.x,
#                    robot_frame_cmd_pos.point.y)
#            rospy.loginfo(msg)
            rospy.loginfo(robot_frame_cmd_pos)
            rospy.loginfo(ps)
#            err_msg = ('xErr: {:.3f} yErr: {:.3f} wErr: {:.3f}').format(
#                    xErr, yErr, wErr)
#            rospy.loginfo(err_msg)
#            rospy.loginfo(cmd_pose)
#            rospy.loginfo(cmd_vel)

        pub.publish(cmd_vel)

    except tf.ExtrapolationException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    pidX = PID(Kp=1, Ki=0, Kd=0, output_limits=(-1., 1.),
               sample_time=SAMPLE_TIME)
    pidY = PID(Kp=1, Ki=0, Kd=0, output_limits=(-1., 1.),
               sample_time=SAMPLE_TIME)
    pidW = PID(Kp=0, Ki=0, Kd=0, output_limits=(-100., 100.),
               sample_time=SAMPLE_TIME)

    cmd_pose = [PoseStamped()]
    cmd_pose[0].header.frame_id = 'world'

    pub = rospy.Publisher('/Mambo_0/cmd_vel', Twist, queue_size=10)
    debugPub = rospy.Publisher('/Mambo_0/debug', Twist, queue_size=10)
    listener = tf.TransformListener()

    def wrapper(data): return callback(data, pub, pidX, pidY, pidW, cmd_pose,
                                       listener, debugPub)

    def readWrapper(data): return readCmd(data, cmd_pose)

    rospy.Subscriber('/vrpn_client_node/Mambo_0/pose', PoseStamped, wrapper)
    rospy.Subscriber('/Mambo_0/cmd_pose', PoseStamped, readWrapper)

    rospy.spin()
