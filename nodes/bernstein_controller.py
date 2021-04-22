#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 22 10:32:23 2019

@author: ckielasjensen
"""

import sys
import time
sys.path.insert(0, '/home/ckielasjensen/OptimalBezierTrajectoryGeneration/')

from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np
import rospy
import scipy.optimize as sop

import bezier as bez
from optimization import BezOptimization

SAFE_DIST = 1.25 # m
NUM_VEH = 3
MAX_SPEED = 1 # m/s
FINAL_PTS = [   # Ending points in meters
            (2, 1),
            (-3.5, 1),
            (-0.3, 1)
            ]
#FINAL_PTS = [   # Ending points in meters
#            (-3.5, -1),
#            (-0.3, -1),
#            (2, -1)
#            ]
#FINAL_PTS = [   # Ending points in meters
#            (-3.5, -1),
#            (-0.75, -1),
#            (1, -1)
#            ]
#FINAL_PTS = [   # Ending points in meters
#            (-2.3, -1),
#            (2, 1)
#            ]
R = 1.25
FINAL_PTS = [
            (R*np.cos(0), R*np.sin(0)),
            (R*np.cos(2*np.pi/3), R*np.sin(2*np.pi/3)),
            (R*np.cos(4*np.pi/3), R*np.sin(4*np.pi/3))
            ]

OBSTACLES = [(0, 0)]


def area_constraint(x, bezopt):
    """
    """
    cpts = bezopt.reshapeVector(x)


def pose_callback(data, name, pose_dict):
    pose_dict[name] = data

if __name__ == '__main__':
    rospy.init_node('bernstein_controller')

    # Set up subscriers for vehicle positions
    pose_dict = {}
    for i in range(NUM_VEH):
        rospy.Subscriber('/vrpn_client_node/AR_{}/pose'.format(i),
                         PoseStamped,
                         lambda x, i=i: pose_callback(x, 'AR_{}'.format(i),
                         pose_dict))

#    rospy.Subscriber('/vrpn_client_node/AR_0/pose', PoseStamped,
#                     lambda x: pose_callback(x, 'AR_0', pose_dict))
#    rospy.Subscriber('/vrpn_client_node/AR_1/pose', PoseStamped,
#                     lambda x: pose_callback(x, 'AR_1', pose_dict))
#    rospy.Subscriber('/vrpn_client_node/AR_2/pose', PoseStamped,
#                     lambda x: pose_callback(x, 'AR_2', pose_dict))

    # Set up publishers for sending commands
    pubListPose = []
    pubListTwist = []
    for i in range(NUM_VEH):
        pubNamePose = '/AR_{}/cmd_pose'.format(i)
        pubNameTwist = '/AR_{}/cmd_twist'.format(i)
        pubListPose.append(rospy.Publisher(pubNamePose, PoseStamped,
                                           queue_size=10))
        pubListTwist.append(rospy.Publisher(pubNameTwist, TwistStamped,
                                            queue_size=10))

    # Sleep for a moment to allow ROS to get everything set up
    time.sleep(1)

    while len(pose_dict.keys()) < NUM_VEH:
        print(pose_dict.keys())
        time.sleep(0.1)

    # Get the initial points of the vehicles
    initPts = []
    for i in range(NUM_VEH):
        vehicleName = 'AR_{}'.format(i)
        curVeh = pose_dict[vehicleName]
        initPts.append((curVeh.pose.position.x, curVeh.pose.position.y))

    print(initPts)

    # =========================================================================
    # Bernstein Optimal Trajectories
    # =========================================================================
    bezopt = BezOptimization(numVeh=NUM_VEH,
                             dimension=2,
                             degree=5,
                             minimizeGoal='TimeOpt',
                             maxSep=SAFE_DIST,
                             maxSpeed=MAX_SPEED,
#                             maxAngRate=1,
                             initPoints=initPts,
                             finalPoints=FINAL_PTS,
                             initSpeeds=[0]*NUM_VEH,
                             finalSpeeds=[0]*NUM_VEH,
                             initAngs=[np.pi/2]*NUM_VEH,
                             finalAngs=[np.pi/2]*NUM_VEH,
                             pointObstacles=OBSTACLES
                             )

    xGuess = bezopt.generateGuess(std=0)
    ineqCons = [{'type': 'ineq', 'fun': bezopt.temporalSeparationConstraints},
                {'type': 'ineq', 'fun': bezopt.maxSpeedConstraints},
                {'type': 'ineq', 'fun': lambda x: x[-1]-1}]


    print('starting')
    startTime = time.time()
    results = sop.minimize(
                bezopt.objectiveFunction,
                x0=xGuess,
                method='SLSQP',
                constraints=ineqCons,
                options={'maxiter': 250,
                         'disp': True,
                         'iprint': 2}
                )
    endTime = time.time()

    while not results.success:
        xGuess = bezopt.generateGuess(std=3)
        print('Optimization failed, trying again...')
        startTime = time.time()
        results = sop.minimize(
                    bezopt.objectiveFunction,
                    x0=xGuess,
                    method='SLSQP',
                    constraints=ineqCons,
                    options={'maxiter': 250,
                             'disp': False,
                             'iprint': 2}
                    )
        endTime = time.time()

    print('---')
    print('Computation Time: {}'.format(endTime - startTime))
    print('---')

    numVeh = bezopt.model['numVeh']
    dim = bezopt.model['dim']
    maxSep = bezopt.model['maxSep']

    cpts = bezopt.reshapeVector(results.x)
    tf = results.x[-1]

    curves = []
    for i in range(numVeh):
        curves.append(bez.Bezier(cpts[i*dim:(i+1)*dim],
                                      tau=np.linspace(0, tf, 101),
                                        tf=tf))
    # =========================================================================

    # Create trajectories
    trajectories = [c.curve for c in curves]
    trajectories_dot = [c.diff().curve for c in curves]

    # Run until the final point is reached
    curveIdx = 0
    lastTime = time.time()
    cmd_pose = PoseStamped()
    cmd_pose.header.frame_id = 'world'
    cmd_twist = TwistStamped()
    cmd_twist.header.frame_id = 'world'
    while not rospy.is_shutdown():
        if (time.time() - lastTime > tf/101.0):
            print(curveIdx)
            lastTime = time.time()
            for i in range(NUM_VEH):
                cmd_pose.pose.position.x = trajectories[i][0, curveIdx]
                cmd_pose.pose.position.y = trajectories[i][1, curveIdx]

                cmd_twist.twist.linear.x = trajectories_dot[i][0, curveIdx]
                cmd_twist.twist.linear.y = trajectories_dot[i][1, curveIdx]

                cur_time = rospy.Time.now()
                cmd_pose.header.stamp = cur_time
                cmd_twist.header.stamp = cur_time

                pubListPose[i].publish(cmd_pose)
                pubListTwist[i].publish(cmd_twist)
            curveIdx += 1
        if curveIdx > 100:
            break

    for i, traj in enumerate(trajectories):
        print('Trajectory {}:\n{}\n'.format(i, traj))

    np.save('Take7', trajectories)

