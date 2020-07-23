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
    init_orientation = p.getQuaternionFromEuler([math.pi / 2.0, 0, math.pi / 2.0])
    quadruped = p.loadURDF("laikago/laikago_toes_limits.urdf",
                           init_position, init_orientation, useFixedBase=True)
    # quadruped = p.loadURDF("mini_cheetah/mini_cheetah.urdf",
    #                        init_position, init_orn, useFixedBase=False)
    p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])
    p.setRealTimeSimulation(1)
    # startPos = [0.02, -0.78, 1.74, 0.02, -0.78,
    #             1.74, -0.02, -0.78, 1.74, -0.02, -0.78, 1.74]
    startPos = [[ -0.13469, 0.19749, -0.98728, -0.29301, -0.20471, -1.23984, -0.23355, 0.42011, -1.21791, -0.18938, 0.26441, -0.94834],
  [-0.14512, 0.26054, -0.95961, -0.29102, -0.36855, -1.18856, -0.27333, 0.31933, -1.19782, -0.16195, 0.32940, -0.92682],
  [-0.15306, 0.30261, -0.90729, -0.25868, -0.48532, -1.10858, -0.30688, 0.20205, -1.13190, -0.13235, 0.37537, -0.87545],
  [-0.16337, 0.32781, -0.83305, -0.22920, -0.55950, -1.03486, -0.33973, 0.07726, -1.03591, -0.10455, 0.40886, -0.80869],
  [-0.16965, 0.34171, -0.74659, -0.23311, -0.60282, -0.94829, -0.36301, -0.03726, -0.92726, -0.07940, 0.43395, -0.73493],
  [-0.17625, 0.35017, -0.65978, -0.19650, -0.61928, -0.86229, -0.37492, -0.13853, -0.82065, -0.06165, 0.46502, -0.67749],
  [-0.17989, 0.35836, -0.58379, -0.11653, -0.58522, -0.80736, -0.37301, -0.22084, -0.73131, -0.05417, 0.51236, -0.66087],
  [-0.18044, 0.36833, -0.54932, -0.08977, -0.53380, -0.76223, -0.36188, -0.28908, -0.66566, -0.05688, 0.56185, -0.67457],
  [-0.17787, 0.39126, -0.61386, -0.09028, -0.47221, -0.71330, -0.33558, -0.32496, -0.62603, -0.06434, 0.61170, -0.71491],
  [-0.16933, 0.42446, -0.72625, -0.08722, -0.39813, -0.70583, -0.30456, -0.33030, -0.61354, -0.07068, 0.66141, -0.78657],
  [-0.17410, 0.44456, -0.86169, -0.08785, -0.30924, -0.75960, -0.27382, -0.31084, -0.62046, -0.07733, 0.69563, -0.87640],
  [-0.18090, 0.44405, -1.01111, -0.09562, -0.21583, -0.83420, -0.26601, -0.24525, -0.67550, -0.08763, 0.71085, -0.97991],
  [-0.19220, 0.40517, -1.15108, -0.10542, -0.11626, -0.90293, -0.27122, -0.14297, -0.76737, -0.10005, 0.70747, -1.08610],
  [-0.20451, 0.30068, -1.25100, -0.12191, -0.01085, -0.96244, -0.27744, -0.03505, -0.84941, -0.11237, 0.67923, -1.17895],
  [-0.21398, 0.14288, -1.31734, -0.13083, 0.09313, -1.00631, -0.27792, 0.05942, -0.90905, -0.12975, 0.62727, -1.25950],
  [-0.20933, -0.03553, -1.31434, -0.13410, 0.18246, -1.01992, -0.26713, 0.15417, -0.95626, -0.15533, 0.55131, -1.31573],
  [-0.18514, -0.20865, -1.24941, -0.13137, 0.24918, -1.00930, -0.24675, 0.23442, -0.97250, -0.18472, 0.44756, -1.32951],
  [-0.15200, -0.35830, -1.14094, -0.12812, 0.29595, -0.98447, -0.22266, 0.30016, -0.96332, -0.21647, 0.32512, -1.29471],
  [-0.10030, -0.47457, -0.99850, -0.12894, 0.32647, -0.93602, -0.19455, 0.35509, -0.93407, -0.24689, 0.19552, -1.21388],
  [-0.07595, -0.54361, -0.90392, -0.13124, 0.34496, -0.86624, -0.16334, 0.39909, -0.89050, -0.27480, 0.06162, -1.09754],
  [-0.07796, -0.59147, -0.83841, -0.13472, 0.35335, -0.78171, -0.13330, 0.43503, -0.83421, -0.29635, -0.06430, -0.96968],
  [-0.02444, -0.57695, -0.76569, -0.14084, 0.35431, -0.68742, -0.11056, 0.45942, -0.76450, -0.31035, -0.17816, -0.84233],
  [0.03306, -0.51857, -0.74178, -0.14370, 0.34841, -0.59691, -0.09653, 0.48009, -0.70593, -0.31628, -0.27009, -0.73351],
  [0.03693, -0.45715, -0.71912, -0.14406, 0.33574, -0.54469, -0.09372, 0.50782, -0.68224, -0.31207, -0.34200, -0.66205],
  [0.02213, -0.38793, -0.73939, -0.14644, 0.34990, -0.58431, -0.10378, 0.53688, -0.68518, -0.29773, -0.36876, -0.64115],
  [-0.00440, -0.30834, -0.79874, -0.15446, 0.38900, -0.69914, -0.12304, 0.56939, -0.71815, -0.27581, -0.36458, -0.64736],
  [-0.01870, -0.22990, -0.84639, -0.17108, 0.43025, -0.84662, -0.14480, 0.60458, -0.77773, -0.24323, -0.34274, -0.65512],
  [-0.03190, -0.13705, -0.89790, -0.19256, 0.44180, -1.00410, -0.16929, 0.62965, -0.84436, -0.21962, -0.29603, -0.68556],
  [-0.05247, -0.04078, -0.94286, -0.22764, 0.38617, -1.12246, -0.19891, 0.64651, -0.92152, -0.21120, -0.20773, -0.75772],
  [-0.06905, 0.04676, -0.96850, -0.26272, 0.27126, -1.20372, -0.23094, 0.65590, -1.00493, -0.20474, -0.09638, -0.84458],
  [-0.08727, 0.13061, -0.98216, -0.30963, 0.11052, -1.24698, -0.26064, 0.64843, -1.08140, -0.19889, 0.00708, -0.92231],
  [-0.09822, 0.20722, -0.98374, -0.34176, -0.06244, -1.25238, -0.28866, 0.62110, -1.14684, -0.18915, 0.09785, -0.97609],
  [-0.10972, 0.27104, -0.96950, -0.35302, -0.23058, -1.21682, -0.31460, 0.57537, -1.20075, -0.16983, 0.18096, -1.01378]]
    footFR_index = 3
    footFL_index = 7
    footHR_index = 11
    footHL_index = 15
    while 1:
        for ii in range(33):
            # print(ii)
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
            pos = p.getLinkState(quadruped, 3)
            print([pos[0][0], pos[0][1], pos[0][2]])


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
