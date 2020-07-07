#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import threading
import numpy as np
import rospy
import rospkg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from kinematic_model import robotKinematics
from gaitPlanner import trotGait
import pybullet as p
import pybullet_data
from pybullet_debuger import pybulletDebug

SIM = False


class RobotParm:
    def __init__(self):
        self.gaitLength = 0
        self.gaitYaw = 0
        self.rotSpeed = 0
        self.robotX = 0
        self.robotY = 0
        self.robotZ = 0
        self.robotRoll = 0
        self.robotPitch = 0
        self.robotYaw = 0
        self.T = 0.25
        self.freq = 200

        self.footFR_index = 3
        self.footFL_index = 7
        self.footHR_index = 11
        self.footHL_index = 15
        self.compensateSim = [1, -1, -1]
        self.compensateReal = [-1, -1, -1, 1, -1, -1, -1, 1, 1, 1, 1, 1]

        self.Xdist = 0.38
        self.Ydist = 0.1161*2
        self.height = 0.26
        self.bodytoFeet0 = np.matrix([[self.Xdist/2, -self.Ydist/2, -self.height],
                                      [self.Xdist/2,  self.Ydist/2, -self.height],
                                      [-self.Xdist/2, -self.Ydist/2, -self.height],
                                      [-self.Xdist/2,  self.Ydist/2, -self.height]])
        self.offset = np.array([0.5, 0., 0., 0.5])


def ThreadJob():
    rospy.spin()


def velCmdCallBack(msg):
    linearX = msg.linear.x
    linearY = -msg.linear.y
    robParm.rotSpeed = msg.angular.x * 0.25
    if math.fabs(linearX) < 0.1 and math.fabs(linearY) < 0.1:
        robParm.gaitLength = 0
        robParm.gaitYaw = 0
    else:
        robParm.gaitLength = math.sqrt(linearX*linearX + linearY*linearY)*0.5
        robParm.gaitYaw = math.degrees(math.atan2(linearY, linearX))


def PoseCmdCallBack(msg):
    robParm.robotRoll += 0.1*msg.angular.x
    if robParm.robotRoll > math.pi/4:
        robParm.robotRoll = math.pi/4
    elif robParm.robotRoll < -math.pi/4:
        robParm.robotRoll = -math.pi/4
    robParm.robotPitch += 0.1*msg.angular.y
    if robParm.robotPitch > math.pi/4:
        robParm.robotPitch = math.pi/4
    elif robParm.robotPitch < -math.pi/4:
        robParm.robotPitch = -math.pi/4
    robParm.robotYaw += 0.1*msg.angular.z
    if robParm.robotYaw > math.pi/4:
        robParm.robotYaw = math.pi/4
    elif robParm.robotYaw < -math.pi/4:
        robParm.robotYaw = -math.pi/4


def InitSimEnv():
    global quadruped
    global pybulletDebug

    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    pybulletDebug = pybulletDebug()
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    init_position = [0, 0, 0.5]
    init_orn = [0, 0, 0, 1]
    path = rospkg.RosPack().get_path("quadruped_ctrl")
    print("current path = " + path)
    quadruped = p.loadURDF(path + "/src/quadruped_robot.urdf",
                           init_position, init_orn, useFixedBase=False)
    p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])
    p.setRealTimeSimulation(1)
    startPos = [0.02, -0.78, 1.74, 0.02, -0.78,
                1.74, -0.02, -0.78, 1.74, -0.02, -0.78, 1.74]
    footFR_index = 3
    footFL_index = 7
    footHR_index = 11
    footHL_index = 15
    for i in range(0, footFR_index):
        p.setJointMotorControl2(
            quadruped, i, p.POSITION_CONTROL, startPos[i])  # , force = 20)
    for i in range(footFR_index + 1, footFL_index):
        p.setJointMotorControl2(
            quadruped, i, p.POSITION_CONTROL, startPos[i-1])  # , force = 20)
    for i in range(footFL_index + 1, footHR_index):
        p.setJointMotorControl2(
            quadruped, i, p.POSITION_CONTROL, startPos[i-2])  # , force = 20)
    for i in range(footHR_index + 1, footHL_index):
        p.setJointMotorControl2(
            quadruped, i, p.POSITION_CONTROL, startPos[i - 3])  # , force = 20)


def QuadrupedCtrl():
    global quadruped

    rate = rospy.Rate(robParm.freq)
    setJSMsg = JointState()

    while not rospy.is_shutdown():
        start = pybulletDebug.cam_and_robotstates(SIM, quadruped, robParm)
        if start:
            bodytoFeet = trot.loop(robParm)
            position = []
            for i in range(4):
                for j in range(3):
                    position.append(bodytoFeet[i][j].tolist())
            FR_angles, FL_angles, HR_angles, HL_angles, _ = robotKinematics.solve(
                bodytoFeet, robParm)

            for i in range(0, robParm.footFR_index):
                p.setJointMotorControl2(quadruped, i, p.POSITION_CONTROL,
                                        FR_angles[i - robParm.footFR_index]*robParm.compensateSim[i])  # , force = 20)
            for i in range(robParm.footFR_index + 1, robParm.footFL_index):
                p.setJointMotorControl2(quadruped, i, p.POSITION_CONTROL,
                                        FL_angles[i - robParm.footFL_index]*robParm.compensateSim[i-robParm.footFR_index - 1])  # , force = 20)
            for i in range(robParm.footFL_index + 1, robParm.footHR_index):
                p.setJointMotorControl2(quadruped, i, p.POSITION_CONTROL,
                                        HR_angles[i - robParm.footHR_index]*robParm.compensateSim[i-robParm.footFL_index - 1])  # , force = 20)
            for i in range(robParm.footHR_index + 1, robParm.footHL_index):
                p.setJointMotorControl2(quadruped, i, p.POSITION_CONTROL,
                                        HL_angles[i - robParm.footHL_index]*robParm.compensateSim[i-robParm.footHR_index - 1])  # , force = 20)

            angle = FL_angles.tolist()
            angle.extend(HL_angles.tolist())
            angle.extend(FR_angles.tolist())
            angle.extend(HR_angles.tolist())

            setJSMsg.header.stamp = rospy.Time.now()
            setJSMsg.name = ["abduct_fl", "thigh_fl", "knee_fl", "abduct_hl", "thigh_hl", "knee_hl",
                             "abduct_fr", "thigh_fr", "knee_fr", "abduct_hr", "thigh_hr", "knee_hr"]
            for i in range(12):
                angle[i] = angle[i] * robParm.compensateReal[i]
            print(angle)
            setJSMsg.position = angle
            setJsPub.publish(setJSMsg)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('quatruped_ctrl', anonymous=True)
    setJsPub = rospy.Publisher('/set_js', JointState, queue_size=10)
    rospy.Subscriber("/cmd_vel", Twist, velCmdCallBack)
    rospy.Subscriber("/cmd_pose", Twist, PoseCmdCallBack)
    add_thread = threading.Thread(target=ThreadJob)
    add_thread.start()

    robotKinematics = robotKinematics()
    trot = trotGait()
    robParm = RobotParm()

    InitSimEnv()

    QuadrupedCtrl()
