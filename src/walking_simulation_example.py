#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import threading
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from kinematic_model import robotKinematics
from gaitPlanner import trotGait
import pybullet as p
import pybullet_data
from pybullet_debuger import pybulletDebug  


robotKinematics = robotKinematics()
trot = trotGait()

def ThreadJob():
    rospy.spin()

def velCmdCallBack(msg):
    global gaitLength
    global gaitYaw
    global rotSpeed

    linearX = msg.linear.x
    linearY = -msg.linear.y
    rotSpeed = msg.angular.x * 0.25
    # angularY = msg.angular.y

    if math.fabs(linearX)<0.1 and math.fabs(linearY)<0.1:
        gaitLength = 0
        gaitYaw = 0
    else:
        gaitLength = math.sqrt(linearX*linearX + linearY*linearY)*0.5
        gaitYaw = math.degrees(math.atan2(linearY, linearX))

def QuadrupedCtrl():
    global gaitLength
    global gaitYaw
    global rotSpeed

    footFR_index = 3
    footFL_index = 7
    footHR_index = 11
    footHL_index = 15   

    #robot properties
    """initial foot position"""
    #foot separation (Ydist = 0.16 -> tetta=0) and distance to floor
    compensate = [1, -1, -1]
    Xdist = 0.38
    Ydist = 0.1161*2
    height = 0.25
    #body frame to foot frame vector
    bodytoFeet0 = np.matrix([[ Xdist/2 , -Ydist/2 , -height],
                            [ Xdist/2 ,  Ydist/2 , -height],
                            [-Xdist/2 , -Ydist/2 , -height],
                            [-Xdist/2 ,  Ydist/2 , -height]])

    offset = np.array([0.5 , 0. , 0. , 0.5]) #defines the offset between each foot step in this order (FR,FL,HR,HL)
    pos = np.array([0, 0, 0])
    orn = np.array([0, 0, 0])
    gaitLength = 0
    gaitYaw = 0
    rotSpeed = 0
    T = 0.25

    freq = 200
    rate = rospy.Rate(freq)
    setJSMsg = JointState()
    while not rospy.is_shutdown():
        # pos , orn , gaitLength , gaitYaw , rotSpeed , T = pybulletDebug.cam_and_robotstates(quadruped)  
        print([gaitLength, gaitYaw, rotSpeed])
        bodytoFeet = trot.loop(gaitLength , gaitYaw , rotSpeed , T , offset , bodytoFeet0)
        position = []
        for i in range(4):
            for j in range(3):
                # print(bodytoFeet[i][j].tolist())
                position.append(bodytoFeet[i][j].tolist())
        # print(position)
        FR_angles, FL_angles, HR_angles, HL_angles , _ = robotKinematics.solve(orn , pos , bodytoFeet)

        #move movable joints
        for i in range(0, footFR_index):
            p.setJointMotorControl2(quadruped, i, p.POSITION_CONTROL, FR_angles[i - footFR_index]*compensate[i])
        for i in range(footFR_index + 1, footFL_index):
            p.setJointMotorControl2(quadruped, i, p.POSITION_CONTROL, FL_angles[i - footFL_index]*compensate[i-footFR_index - 1])
        for i in range(footFL_index + 1, footHR_index):
            p.setJointMotorControl2(quadruped, i, p.POSITION_CONTROL, HR_angles[i - footHR_index]*compensate[i-footFL_index - 1])
        for i in range(footHR_index + 1, footHL_index):
            p.setJointMotorControl2(quadruped, i, p.POSITION_CONTROL, HL_angles[i - footHL_index]*compensate[i-footHR_index - 1])

        angle = FL_angles.tolist() 
        angle.extend(HL_angles.tolist())
        angle.extend(FR_angles.tolist())
        angle.extend(HR_angles.tolist())
        # print(angle)

        setJSMsg.header.stamp = rospy.Time.now()
        setJSMsg.name = ["abduct_fl", "thigh_fl", "knee_fl", "abduct_hl", "thigh_hl", "knee_hl",
                            "abduct_fr", "thigh_fr", "knee_fr", "abduct_hr", "thigh_hr", "knee_hr"]
        setJSMsg.position = angle
        setJsPub.publish(setJSMsg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('quatruped_ctrl', anonymous=True)
    setJsPub = rospy.Publisher('/set_js', JointState, queue_size=10)
    rospy.Subscriber("/cmd_vel", Twist, velCmdCallBack)
    add_thread = threading.Thread(target=ThreadJob)
    add_thread.start()

    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    pybulletDebug = pybulletDebug()
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-9.81)
    # cubeStartPos = [0,0,0.2]
    # cubeStartOrn = [0, 0, 0, 1]
    # FixedBase = False #if fixed no plane is imported
    # if (FixedBase == False):
    p.loadURDF("plane.urdf")
    # boxId = p.loadURDF("/home/derek/ros_workspace/quadruped_ws/src/4-legged-robot-model/src/4leggedRobot.urdf",cubeStartPos, baseOrientation = cubeStartOrn, useFixedBase=FixedBase)
    init_position = [0, 0, 0.5]
    init_orn = [0, 0, 0, 1]
    quadruped = p.loadURDF("mini_cheetah/mini_cheetah.urdf", init_position, init_orn, useFixedBase=False)
    p.resetDebugVisualizerCamera(0.2, 45, -30, [1, -1, 1])
    p.setRealTimeSimulation(1)

    QuadrupedCtrl()

    


    

    
        
    



    
