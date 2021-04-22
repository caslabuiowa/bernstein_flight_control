#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 15 10:08:24 2019

@author: ckielasjensen
"""

from geometry_msgs.msg import Twist
from pyparrot.Minidrone import Mambo
import rospy
import time
import numpy as np

TESTING = False

DURATION = None #1e-6 # 1./20.
#MAMBO_ADDR = 'd0:3a:e4:e6:e6:22'  # Mambo_0
#MAMBO_ADDR = 'e0:14:fa:a6:3d:fd'  # Mambo_1
MAMBO_ADDR = 'd0:3a:ae:86:e6:23'  # Mambo with gripper


class FakeMambo():
    def __init__(self, *args, **kwargs):
        pass

    def fly_direct(self, *args, **kwargs):
        for kw in kwargs:
            rospy.loginfo('{}: {}'.format(kw, kwargs[kw]))

    def connect(self, *args, **kwargs):
        rospy.loginfo('CALLED CONNECT')
        return True

    def safe_land(self, *args, **kwargs):
        rospy.loginfo('CALLED SAFE_LAND')

    def smart_sleep(self, *args, **kwargs):
        rospy.loginfo('CALLED SMART_SLEEP')

    def disconnect(self, *args, **kwargs):
        rospy.loginfo('CALLED DISCONNECT')

    def ask_for_state_update(self):
        rospy.loginfo('ASKING FOR STATE INFO')

    def safe_takeoff(self, *args, **kwargs):
        rospy.loginfo('TAKING OFF')


def callback(data, flight_cmd):
    roll = data.linear.y
    pitch = data.linear.x
    yaw = data.angular.z
    vertical = data.linear.z
#    rospy.loginfo('Roll: {}, Pitch: {}, Yaw: {}, Vertical: {}'.format(roll,
#                  pitch, yaw, vertical))

    flight_cmd['roll'] = roll
    flight_cmd['pitch'] = pitch
    flight_cmd['yaw'] = yaw
    flight_cmd['vertical'] = vertical


def cb(data):
    data.update
    print(data.flying_state)


if __name__ == '__main__':

    if TESTING:
        mambo = FakeMambo()
    else:
        mambo = Mambo(MAMBO_ADDR, use_wifi=False)

    flight_cmd = {
            'roll': 0,
            'pitch': 0,
            'yaw': 0,
            'vertical': 0
            }

    rospy.loginfo('Connecting...')
    success = mambo.connect(num_retries=3)
    rospy.loginfo('Connected: {}'.format(success))
    
    mambo.set_user_sensor_callback(cb, mambo.sensors)

    def wrapper(data): return callback(data, flight_cmd)

    rospy.init_node('bt_relay', anonymous=True)
    rospy.Subscriber('Mambo_0/cmd_vel', Twist, wrapper, queue_size=10)

    try:
        if success:
            print('Getting state information...')
            mambo.smart_sleep(2)
            mambo.ask_for_state_update()
            mambo.smart_sleep(2)
#            mambo.set_max_tilt(360)

            print('Taking off!')
            mambo.safe_takeoff(5)
            
            count = 0

            while not rospy.is_shutdown():
#                msg = 'Roll: {}, Pitch: {}, Yaw: {}, Vertical: {}'.format(
#                        flight_cmd['roll'],
#                        flight_cmd['pitch'],
#                        flight_cmd['yaw'],
#                        flight_cmd['vertical'])
#                rospy.loginfo(msg)
#                print(count)
#                count+=1
                mambo.fly_direct(
                        roll=flight_cmd['roll'],
                        pitch=flight_cmd['pitch'],
                        yaw=flight_cmd['yaw'],
                        vertical_movement=flight_cmd['vertical'],
                        duration=None)
                

    finally:
        rospy.loginfo('Landing...')
        mambo.safe_land(5)
        mambo.smart_sleep(5)
        rospy.loginfo('Disconnecting')
        mambo.disconnect()
