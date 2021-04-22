#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 22 10:19:49 2019

#telnet into the drone (telnet 192.168.1.1)
#paste the following command:

killall udhcpd;
iwconfig ath0 mode managed essid [ssid];
ifconfig ath0 [wanted ip] netmask 255.255.255.0 up;

#ex.
killall udhcpd;
iwconfig ath0 mode managed essid Roflcopter;
ifconfig ath0 192.168.43.201 netmask 255.255.255.0 up;

Connect the drone to the wifi network CAS_Lab by first connecting directly to
the drone then run:
echo "/data/wifi.sh" | telnet 192.168.1.1
After running the above command, the drones internet will go down and then
connect to the CAS_Lab network. Make sure the CAS_Lab network doesn't have
a password on it.

@author: ckielasjensen
"""

import time

from geometry_msgs.msg import PoseStamped, PointStamped, Twist
import rospy
from std_msgs.msg import Empty
from simple_pid import PID
import tf

from ardrone_autonomy.msg import Navdata
from drone_status import DroneStatus

TESTING = False
SAMPLE_TIME = 0.02   # None   # 1.0/100
WAIT_DURATION = 1.0  # 10*SAMPLE_TIME
COMMAND_PERIOD = 10  # ms


class ARDrone(object):
    """A basic AR Drone 2.0 controller class

    This drone controller class was copied from Mike Hamer's ardrone_tutorials
    on GitHub at https://github.com/mikehamer/ardrone_tutorials.

    This class implements basic control functionality. It can command
    takeoff/landing/emergency as well as drone movement. It also tracks the
    drone state based on navdata feedback.
    """
    def __init__(self):
        # Holds the current drone status
        self._status = -1

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and
        # call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('ardrone/navdata',
                                           Navdata,
                                           self.ReceiveNavdata)

        # Allow the controller to publish to the takeoff, land and reset topics
        self._pubLand    = rospy.Publisher('ardrone/land',
                                           Empty, queue_size=10)
        self._pubTakeoff = rospy.Publisher('ardrone/takeoff',
                                           Empty, queue_size=10)
        self._pubReset   = rospy.Publisher('ardrone/reset',
                                           Empty, queue_size=10)

        # Allow the controller to publish to the /cmd_vel topic
        self._pubCommand = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Setup regular publishing of control packets
        self._command = Twist()
        self._commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),
                                        self._SendCommand)

        # Land the drone if we are shutting down
        rospy.on_shutdown(self.SendLand)

    def ReceiveNavdata(self, navdata):
        # Although there is a lot of data in this packet, we're only interested
        # in the state at the moment
        self._status = navdata.state

    def SendTakeoff(self):
        # Send a takeoff message to the ardrone driver
        # Note we only send a takeoff message if the drone is landed - an
        # unexpected takeoff is not good!
        if(self._status == DroneStatus.Landed):
            self._pubTakeoff.publish(Empty())

    def SendLand(self):
        # Send a landing message to the ardrone driver
        # Note we send this in all states, landing can do no harm
        self._pubLand.publish(Empty())

    def SendEmergency(self):
        # Send an emergency (or reset) message to the ardrone driver
        self._pubReset.publish(Empty())

    def SendReset(self):
        self.ReceiveNavdata()


    def SetCommand(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        # Called by the main program to set the current command
        self._command.linear.x  = pitch
        self._command.linear.y  = roll
        self._command.linear.z  = z_velocity
        self._command.angular.z = yaw_velocity

    def _SendCommand(self, event):
        # The previously set command is then sent out periodically if the drone
        # is flying
#        self._pubCommand.publish(self._command)
        if (self._status == DroneStatus.Flying or
            self._status == DroneStatus.GotoHover or
            self._status == DroneStatus.Hovering):
              self._pubCommand.publish(self._command)


def readCmd(data, cmd_pose):
    cmd_pose[0] = data
    cmd_pose[0].header.frame_id = 'world'


def callback(data, ardrone, pidX, pidY, pidW, cmd_pose, listener, debugPub):
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

#    print('Roll: {:.5f}, Pitch: {:.5f}, Yaw: {:.5f}'.format(roll, pitch, yaw))

    try:
        ros_time_now = rospy.Time.now()

        ps = PointStamped()
        ps.header = cmd_pose[0].header
        ps.point = cmd_pose[0].pose.position
        ps.header.stamp = ros_time_now

        droneName = 'AR_{}'.format(rospy.get_namespace()[-2])
        listener.waitForTransform('world', droneName, ros_time_now,
                                  rospy.Duration(WAIT_DURATION))
        robot_frame_cmd_pos = listener.transformPoint(droneName, ps)

        if pidX.auto_mode:
            x_cmd = -pidX(-robot_frame_cmd_pos.point.x)
            y_cmd = pidY(-robot_frame_cmd_pos.point.y)
            w_cmd = pidW(yaw)

            cmd_vel = Twist()
            cmd_vel.linear.x = x_cmd
            cmd_vel.linear.y = y_cmd
            cmd_vel.angular.z = w_cmd

            if cmd_pose[0].header.frame_id == 'world':
                ardrone.SetCommand(x_cmd, y_cmd, w_cmd, 0)
#        else:
#            x_cmd = 0
#            y_cmd = 0
#            w_cmd = 0

#        cmd_vel = Twist()
#        cmd_vel.linear.x = x_cmd
#        cmd_vel.linear.y = y_cmd
#        cmd_vel.angular.z = w_cmd
#
#        ardrone.SetCommand(x_cmd, y_cmd, w_cmd, 0)

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

#        pub.publish(cmd_vel)

    except tf.ExtrapolationException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    namespace = rospy.get_namespace()
    print(namespace)

    # 0.35, 0.05, 0.18
    pidX = PID(Kp=0.35, Ki=0.05, Kd=0.18,output_limits=(-1., 1.),
               sample_time=SAMPLE_TIME, auto_mode=False)
    pidY = PID(Kp=0.35, Ki=0.05, Kd=0.18, output_limits=(-1., 1.),
               sample_time=SAMPLE_TIME, auto_mode=False)
    pidW = PID(Kp=0, Ki=0, Kd=0, output_limits=(-1., 1.),
               sample_time=SAMPLE_TIME, auto_mode=False)

    ardrone = ARDrone()

    cmd_pose = [PoseStamped()]
    cmd_pose[0].header.frame_id = 'world'

    debugPub = rospy.Publisher('debug', Twist, queue_size=10)
    listener = tf.TransformListener()

    def wrapper(data): return callback(data, ardrone, pidX, pidY, pidW,
                                       cmd_pose, listener, debugPub)

    def readWrapper(data): return readCmd(data, cmd_pose)

    vrpnTopic = '/vrpn_client_node/AR_{}/pose'.format(namespace[-2])
    print(vrpnTopic)
    rospy.Subscriber(vrpnTopic, PoseStamped, wrapper)
    rospy.Subscriber('cmd_pose', PoseStamped, readWrapper)

    print('Getting initial position...')
    cmd_pose[0] = rospy.wait_for_message(vrpnTopic, PoseStamped)

    print('Taking off...')
    time.sleep(3)
    ardrone.SendEmergency()
    time.sleep(0.5)
    ardrone.SendTakeoff()
    time.sleep(3)
    print('Flying...')
    try:
        pidX.auto_mode = True
        pidY.auto_mode = True
        pidW.auto_mode = True
        rospy.spin()
    finally:
        print('\nLanding...')
        ardrone.SendLand()
        print('Landed.')
