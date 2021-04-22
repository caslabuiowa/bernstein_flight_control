#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 25 16:29:28 2019

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

from geometry_msgs.msg import PoseStamped, PointStamped, Twist, TwistStamped
import numpy as np
import rospy
from std_msgs.msg import Empty
import tf

from ardrone_autonomy.msg import Navdata
from drone_status import DroneStatus

TESTING = False
SAMPLE_TIME = 0.02   # None   # 1.0/100
WAIT_DURATION = 1.0  # 10*SAMPLE_TIME
COMMAND_PERIOD = 5   # ms
CTRL_FREQ = 100      # Hz


def _quat_to_euler(x, y, z, w):
    euler = tf.transformations.euler_from_quaternion((x, y, z, w), axes='rzyx')
    roll = euler[2]
    pitch = euler[1]
    yaw = euler[0]

    return roll, pitch, yaw


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
        self._pose = PoseStamped()
        self._twist = TwistStamped()

        self._namespace = rospy.get_namespace()

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and
        # call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('ardrone/navdata',
                                           Navdata,
                                           self.ReceiveNavdata)

        vrpnTopicBase = '/vrpn_client_node/AR_{}'.format(self._namespace[-2])
        self.subPose = rospy.Subscriber(vrpnTopicBase+'/pose',
                                        PoseStamped,
                                        self.ReceivePose)
        self.subTwist = rospy.Subscriber(vrpnTopicBase+'/twist',
                                         TwistStamped,
                                         self.ReceiveTwist)

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

    def ReceivePose(self, poseData):
        self.pose = poseData
        self.x = poseData.pose.position.x
        self.y = poseData.pose.position.y
        self.z = poseData.pose.position.z
        quat = (poseData.pose.orientation.x,
                poseData.pose.orientation.y,
                poseData.pose.orientation.z,
                poseData.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat, axes='rzyx')

        self.roll = euler[2]
        self.pitch = euler[1]
        self.yaw = euler[0]
#        self.roll, self.pitch, self.yaw = _quat_to_euler(
#                                                poseData.pose.orientation.x,
#                                                poseData.pose.orientation.y,
#                                                poseData.pose.orientation.z,
#                                                poseData.pose.orientation.w)

    def ReceiveTwist(self, twistData):
        self.twist = twistData

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
        if self._status == DroneStatus.Emergency:
            self.SendEmergency()

    def SetCommand(self, roll=0, pitch=0, yaw_velocity=0, z_velocity=0):
        # Called by the main program to set the current command
        self._command.linear.x  = pitch
        self._command.linear.y  = roll
        self._command.angular.z = yaw_velocity
        self._command.linear.z  = z_velocity

    def SetAutoHover(self, auto_hover_on=False):
        if auto_hover_on:
            self._command.angular.x = 0
            self._command.angular.y = 0
        else:
            self._command.angular.x = 1
            self._command.angular.y = 1

    def SetHover(self):
        self.SetAutoHover(True)
        self.SetCommand()

    def _SendCommand(self, event):
        # The previously set command is then sent out periodically if the drone
        # is flying
#        self._pubCommand.publish(self._command)
        if (self._status == DroneStatus.Flying or
            self._status == DroneStatus.GotoHover or
            self._status == DroneStatus.Hovering):
              self._pubCommand.publish(self._command)


class Controller(object):
    def __init__(self, Kp=0, Ki=0, Kd=0, Kangle=0, Kpw=0, bounds=(-1,1)):
        """
        """
        self._kp = Kp
        self._ki = Ki
        self._kd = Kd
        self._xe_int = 0
        self._ye_int = 0
        self._kangle = Kangle
        self._kpw = Kpw

        self.world_frame_id = 'world'
        self.robot_frame_id = 'AR_{}'.format(rospy.get_namespace()[-2])

        self._cmd_pose = PoseStamped()
        self._cmd_pose.header.frame_id = 'world'
        self._cmd_twist = TwistStamped()
        self._cmd_twist.header.frame_id = 'world'

#        self.droneName = 'AR_{}'.format(rospy.get_namespace()[-2])

        self._tf_listener = tf.TransformListener()
        self._sub_cmd_pose = rospy.Subscriber('cmd_pose',
                                              PoseStamped,
                                              self._updateCmdPose)
        self._sub_cmd_twist = rospy.Subscriber('cmd_twist',
                                               TwistStamped,
                                               self._updateCmdTwist)

#        self.debugPub = rospy.Publisher('debug', TwistStamped, queue_size=10)

        self._last_time = rospy.Time.now()

        self._clamp = lambda n: max(min(bounds[1], n), bounds[0])

    def _updateCmdPose(self, data):
        """
        Assumes the yaw angle is given in the z value of orientation, x, y,
        and w are unused.
        """
        self._cmd_pose = data
        self._cmd_pose.header.frame_id = self.world_frame_id

    def _updateCmdTwist(self, data):
        """

        """
        self._cmd_twist = data
        self._cmd_twist.header.frame_id = self.world_frame_id

    def init_pose(self, topic):
        self._cmd_pose = rospy.wait_for_message(topic, PoseStamped)

    def update(self, ardrone):
        """
        """
        ps = PointStamped()
        ps.header = self._cmd_pose.header
        ps.header.frame_id = 'world'
        ps.point = self._cmd_pose.pose.position
        ros_time_now = rospy.Time.now()
        ps.header.stamp = ros_time_now

        try:
            self._tf_listener.waitForTransform('world',
                                               self.robot_frame_id,
                                               ros_time_now,
                                               rospy.Duration(WAIT_DURATION))
            robot_frame_cmd_pos = self._tf_listener.transformPoint(
                self.robot_frame_id, ps)

            world_frame_cmd_yaw = self._cmd_pose.pose.orientation.z

            robot_xerr = robot_frame_cmd_pos.point.y
            robot_yerr = robot_frame_cmd_pos.point.x
            robot_werr = world_frame_cmd_yaw - ardrone.yaw

            time_now = rospy.Time.now()
            dt = (time_now - self._last_time).to_sec()
            self._xe_int +=  robot_xerr*dt
            self._ye_int += robot_yerr*dt
            self._last_time = time_now

            # Robot velocity in the world
            world_xve = ardrone.twist.twist.linear.x
            world_yve = ardrone.twist.twist.linear.y

            world_xverr = self._cmd_twist.twist.linear.x - world_xve
            world_yverr = self._cmd_twist.twist.linear.y - world_yve

            # Velocity rotation matrix
            wr_theta = ardrone.yaw + np.pi/2
            wr_rot = np.array([[np.cos(wr_theta), np.sin(wr_theta)],
                               [np.sin(wr_theta), -np.cos(wr_theta)]])

            world_verr = np.array([world_xverr, world_yverr], ndmin=2).T
            robot_verr = wr_rot.dot(world_verr)

            xverr = robot_verr[0]
            yverr = robot_verr[1]
            """
            # Convert world velocity to robot's frame
            world_vel = np.array([world_xve, world_yve], ndmin=2).T
            robot_vel = wr_rot.dot(world_vel)
            robot_xve = robot_vel[0]
            robot_yve = robot_vel[1]

            xverr = self._cmd_twist.twist.linear.x - robot_xve
            yverr = self._cmd_twist.twist.linear.y - robot_yve
            """
#            xve = (world_xve*np.cos(ardrone.yaw + np.pi/2) +
#                   world_yve*np.sin(ardrone.yaw + np.pi/2))
#            yve = (world_xve*np.sin(ardrone.yaw + np.pi/2) -
#                   world_yve*np.cos(ardrone.yaw + np.pi/2))

#            debugTwist = TwistStamped()
#            debugTwist.header.stamp = rospy.Time.now()
#            debugTwist.twist.linear.x = xve
#            debugTwist.twist.linear.y = yve
#
#            self.debugPub.publish(debugTwist)

#            yve = ardrone.twist.twist.linear.y*np.sin(ardrone.yaw)
#            print('Yaw: {: .5f}'.format(ardrone.yaw))
#            print('Xve: {: .5f}, Yve: {: .5f}'.format(xve, yve))

            x_cmd = self._clamp(self._kp*robot_xerr + self._ki*self._xe_int +
                                self._kd*xverr - self._kangle*ardrone.pitch)

            y_cmd = -self._clamp(self._kp*robot_yerr + self._ki*self._ye_int +
                                 self._kd*yverr - self._kangle*ardrone.roll)

            w_cmd = self._clamp(self._kpw*robot_werr)

            return x_cmd, y_cmd, w_cmd

        except tf.ExtrapolationException as e:
            rospy.loginfo(e)
            return 0, 0, 0


#def readCmd(data, cmd_pose):
#    cmd_pose[0] = data
#    cmd_pose[0].header.frame_id = 'world'


if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True)
    namespace = rospy.get_namespace()
    rate = rospy.Rate(CTRL_FREQ)
#    print(namespace)

#    # 0.35, 0.05, 0.18
#    pidX = PID(Kp=0.35, Ki=0.05, Kd=0.18,output_limits=(-1., 1.),
#               sample_time=SAMPLE_TIME, auto_mode=False)
#    pidY = PID(Kp=0.35, Ki=0.05, Kd=0.18, output_limits=(-1., 1.),
#               sample_time=SAMPLE_TIME, auto_mode=False)
#    pidW = PID(Kp=0, Ki=0, Kd=0, output_limits=(-1., 1.),
#               sample_time=SAMPLE_TIME, auto_mode=False)

    ardrone = ARDrone()
    # Kp=0.2, Ki=0.05, Kd=0.2, Kangle=0.5 <-- Known good (Ki overshoot)
    # Kp=0.2, Ki=0.01, Kd=0.2, Kangle=0.3, Kpw=0.3 <-- Known good (fast)
    controller = Controller(Kp=0.2, Ki=0.01, Kd=0.2, Kangle=0.3, Kpw=0.5)

#    cmd_pose = [PoseStamped()]
#    cmd_pose[0].header.frame_id = 'world'

#    listener = tf.TransformListener()

#    def wrapper(data): return callback(data, ardrone, pidX, pidY, pidW,
#                                       cmd_pose, listener, debugPub)

#    def readWrapper(data): return readCmd(data, cmd_pose)

#    vrpnTopic = '/vrpn_client_node/AR_{}/pose'.format(namespace[-2])
#    rospy.Subscriber(vrpnTopic, PoseStamped, wrapper)
#    rospy.Subscriber('cmd_pose', PoseStamped, readWrapper)

    print('Getting initial position...')
    vrpnTopic = '/vrpn_client_node/AR_{}/pose'.format(namespace[-2])

    controller.init_pose(vrpnTopic)

#    cmd_pose[0] = rospy.wait_for_message(vrpnTopic, PoseStamped)

    print('Taking off...')
#    time.sleep(3)
    ardrone.SetAutoHover(True)
    time.sleep(0.5)
    ardrone.SendReset()
    time.sleep(0.5)
    ardrone.SendTakeoff()
    print(controller.robot_frame_id)
    time.sleep(3)
#    while ardrone._status != DroneStatus.Flying:
#        pass

    print('Flying...')
    try:
        while not rospy.is_shutdown():
            x_cmd, y_cmd, w_cmd = controller.update(ardrone)
#            print('X: {}, Y: {}'.format(x_cmd, y_cmd))
            ardrone.SetCommand(pitch=x_cmd, roll=y_cmd, yaw_velocity=w_cmd,
                               z_velocity=0.0)
            rate.sleep()
    finally:
        print('\nLanding...')
        ardrone.SendLand()
        print('Landed.')
