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
from simple_pid import PID

SIM = True


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
        self.freq = 1000

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
    robParm.rotSpeed = msg.angular.x * 0.125
    if math.fabs(linearX) < 0.1 and math.fabs(linearY) < 0.1:
        robParm.gaitLength = 0
        robParm.gaitYaw = 0
    else:
        robParm.gaitLength = math.sqrt(linearX*linearX + linearY*linearY)*0.5
        robParm.gaitYaw = math.degrees(math.atan2(linearY, linearX))


def PoseCmdCallBack(msg):
    robParm.robotRoll += 0.1*msg.angular.x
    if robParm.robotRoll > math.pi/6:
        robParm.robotRoll = math.pi/6
    elif robParm.robotRoll < -math.pi/6:
        robParm.robotRoll = -math.pi/6
    robParm.robotPitch += 0.1*msg.angular.y
    if robParm.robotPitch > math.pi/6:
        robParm.robotPitch = math.pi/6
    elif robParm.robotPitch < -math.pi/6:
        robParm.robotPitch = -math.pi/6
    robParm.robotYaw += 0.1*msg.angular.z
    if robParm.robotYaw > math.pi/6:
        robParm.robotYaw = math.pi/6
    elif robParm.robotYaw < -math.pi/6:
        robParm.robotYaw = -math.pi/6

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
                           init_position, init_orn, useFixedBase=True)
    # quadruped = p.loadURDF("mini_cheetah/mini_cheetah.urdf",
    #                        init_position, init_orn, useFixedBase=False)
    p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])
    p.setRealTimeSimulation(1)
    # startPos = [0.02, -0.78, 1.74, 0.02, -0.78,
    #             1.74, -0.02, -0.78, 1.74, -0.02, -0.78, 1.74]
    startPos = [[0.18964, -0.73940, 1.69547, -0.37464, -0.29397, 1.96607, 0.29970, -0.95008, 1.94149, -0.24429, -0.81545, 1.66544],
                            [0.19949, -0.80721, 1.66951, -0.37832, -0.12916, 1.91088, 0.33957, -0.85016, 1.92731, -0.21613, -0.88191, 1.63938],
                            [0.20642, -0.85550, 1.61857, -0.34610, -0.01919, 1.82495, 0.37164, -0.73796, 1.86622, -0.18533, -0.93072, 1.58316],
                            [0.21550, -0.88837, 1.54637, -0.31481, 0.04758, 1.74697, 0.40253, -0.62099, 1.77531, -0.15635, -0.96748, 1.51184],
                            [0.22052, -0.91000, 1.46168, -0.31476, 0.08178, 1.66104, 0.42368, -0.51515, 1.67112, -0.13026, -0.99580, 1.43397],
                            [0.22615, -0.92584, 1.37707, -0.27118, 0.08824, 1.57191, 0.43360, -0.42188, 1.56801, -0.11232, -1.02931, 1.37358],
                            [0.22917, -0.94010, 1.30293, -0.18317, 0.04696, 1.50833, 0.42996, -0.34557, 1.48027, -0.10574, -1.07751, 1.35554],
                            [0.22962, -0.95276, 1.26910, -0.15075, -0.00975, 1.46059, 0.41762, -0.28079, 1.41427, -0.10993, -1.12702, 1.36941],
                            [0.22800, -0.97116, 1.33172, -0.14652, -0.07690, 1.41282, 0.38997, -0.24627, 1.37125, -0.11927, -1.17571, 1.41057],
                            [0.22118, -0.99583, 1.44078, -0.14077, -0.15305, 1.40554, 0.35776, -0.24047, 1.35384, -0.12789, -1.22201, 1.48284],
                            [0.22850, -1.00623, 1.57583, -0.14100, -0.23939, 1.45947, 0.32602, -0.25845, 1.35541, -0.13672, -1.25113, 1.57357],
                            [0.23876, -0.99325, 1.72599, -0.14943, -0.32874, 1.53531, 0.31844, -0.32069, 1.40807, -0.14916, -1.25980, 1.67890],
                            [0.25440, -0.93976, 1.86820, -0.15995, -0.42477, 1.60576, 0.32497, -0.41793, 1.49933, -0.16381, -1.24861, 1.78763],
                            [0.27125, -0.82003, 1.97062, -0.17727, -0.52778, 1.66814, 0.33259, -0.52184, 1.58147, -0.17812, -1.21201, 1.88332],
                            [0.28657, -0.64624, 2.03815, -0.18683, -0.63046, 1.71369, 0.33416, -0.61365, 1.64069, -0.19756, -1.15141, 1.96764],
                            [0.28634, -0.46001, 2.03275, -0.19013, -0.72083, 1.72798, 0.32412, -0.70626, 1.68583, -0.22502, -1.06808, 2.02866],
                            [0.26361, -0.28961, 1.96213, -0.18694, -0.79000, 1.71696, 0.30363, -0.78626, 1.69870, -0.25504, -0.96059, 2.04712],
                            [0.22901, -0.14991, 1.84738, -0.18303, -0.83992, 1.69156, 0.27888, -0.85325, 1.68560, -0.28578, -0.83993, 2.01648],
                            [0.17246, -0.04759, 1.69747, -0.18277, -0.87546, 1.64321, 0.24977, -0.91023, 1.65176, -0.31368, -0.71754, 1.93918],
                            [0.14546, 0.01343, 1.59947, -0.18368, -0.90037, 1.57393, 0.21744, -0.95641, 1.60309, -0.33834, -0.59423, 1.82637],
                            [0.14625, 0.05630, 1.53407, -0.18574, -0.91589, 1.49039, 0.18635, -0.99487, 1.54188, -0.35692, -0.47916, 1.70202],
                            [0.08696, 0.03533, 1.45460, -0.19057, -0.92447, 1.39800, 0.16265, -1.02268, 1.46853, -0.36842, -0.37531, 1.57808],
                            [0.02524, -0.02486, 1.42246, -0.19235, -0.92522, 1.30923, 0.14818, -1.04633, 1.40775, -0.37249, -0.29138, 1.47207],
                            [0.01809, -0.08829, 1.39887, -0.19203, -0.91603, 1.25805, 0.14576, -1.07576, 1.38352, -0.36733, -0.22371, 1.40124],
                            [0.03155, -0.15717, 1.42138, -0.19504, -0.92790, 1.29732, 0.15673, -1.10610, 1.38790, -0.35252, -0.19737, 1.37822],
                            [0.05823, -0.23421, 1.48508, -0.20509, -0.96025, 1.41172, 0.17745, -1.13891, 1.42353, -0.33010, -0.20016, 1.38054],
                            [0.07251, -0.31049, 1.53516, -0.22492, -0.99256, 1.56046, 0.20117, -1.17279, 1.48591, -0.29644, -0.22063, 1.38287],
                            [0.08583, -0.40099, 1.58901, -0.25047, -0.99258, 1.72080, 0.22765, -1.19597, 1.55581, -0.27224, -0.26497, 1.40904],
                            [0.10666, -0.49583, 1.63750, -0.28958, -0.92583, 1.84487, 0.25957, -1.21016, 1.63709, -0.26425, -0.34894, 1.47905],
                            [0.12330, -0.58344, 1.66599, -0.32899, -0.79866, 1.93160, 0.29429, -1.21589, 1.72513, -0.25872, -0.45517, 1.56423],
                            [0.14159, -0.66864, 1.68271, -0.38174, -0.62570, 1.98066, 0.32652, -1.20391, 1.80623, -0.25407, -0.55389, 1.64068],
                            [0.15252, -0.74730, 1.68610, -0.42034, -0.44236, 1.98782, 0.35677, -1.17144, 1.87639, -0.24517, -0.64147, 1.69284],
                            [0.16384, -0.81449, 1.67367, -0.43698, -0.26971, 1.95002, 0.38470, -1.11989, 1.93491, -0.22632, -0.72205, 1.72749]]
    footFR_index = 3
    footFL_index = 7
    footHR_index = 11
    footHL_index = 15
    while 1:
        for ii in range(33):
            print(ii)
            for i in range(0, footFR_index):
                p.setJointMotorControl2(
                    quadruped, i, p.POSITION_CONTROL, startPos[ii][i])  # , force = 20)
            for i in range(footFR_index + 1, footFL_index):
                p.setJointMotorControl2(
                    quadruped, i, p.POSITION_CONTROL, startPos[ii][i-1])  # , force = 20)
            for i in range(footFL_index + 1, footHR_index):
                p.setJointMotorControl2(
                    quadruped, i, p.POSITION_CONTROL, startPos[ii][i-2])  # , force = 20)
            for i in range(footHR_index + 1, footHL_index):
                p.setJointMotorControl2(
                    quadruped, i, p.POSITION_CONTROL, startPos[ii][i - 3])  # , force = 20)
            time.sleep(0.05)


def QuadrupedCtrl():
    global quadruped

    rate = rospy.Rate(robParm.freq)
    setJSMsg = JointState()

    while not rospy.is_shutdown():
        get_orientation = []
        get_euler = []
        start = pybulletDebug.cam_and_robotstates(SIM, quadruped, robParm)
        #start = True
        if start:
            pose_orn = p.getBasePositionAndOrientation(quadruped)
            for i in range(4):
                get_orientation.append(pose_orn[1][i])
            get_euler = p.getEulerFromQuaternion(get_orientation)
            # robParm.robotRoll = -get_euler[0] * 1.5
            # robParm.robotPitch = -get_euler[1] * 0.5
            print(get_euler[0], get_euler[1], get_euler[2])
            bodytoFeet = trot.loop(robParm)
            position = []
            robParm.robotX = pidX(get_euler[1])
            robParm.robotY = pidY(get_euler[0])

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
            # print(angle)
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

    pidX = PID(-0.5, 0.0, 0.001, setpoint=0.)
    pidY = PID(0.5, 0.0, 0.001, setpoint=0.)
    pidX.sample_time = 0.02  # update every 0.02 seconds
    pidY.sample_time = 0.02

    robotKinematics = robotKinematics()
    trot = trotGait()
    robParm = RobotParm()

    InitSimEnv()

    QuadrupedCtrl()
