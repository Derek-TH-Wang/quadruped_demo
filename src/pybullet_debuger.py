#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  2 22:15:21 2020

@author: linux-asd
"""
import pybullet as p
import time
import numpy as np
import sys

class pybulletDebug:
    def __init__(self):
        #Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot) 
        self.cyaw=90
        self.cpitch=-7
        self.cdist=0.66
        time.sleep(0.5)
        
        self.start = p.addUserDebugParameter("start", 1, 0, 0)
        # self.stop = p.addUserDebugParameter("stop", 1, 0, 0)
        self.xId = p.addUserDebugParameter("x" , -0.10 , 0.10 , 0.)
        self.yId = p.addUserDebugParameter("y" , -0.10 , 0.10 , 0.)
        self.zId = p.addUserDebugParameter("z" , -0.10 , 0.10 , 0.)
        self.rollId = p.addUserDebugParameter("roll" , -np.pi/4 , np.pi/4 , 0.)
        self.pitchId = p.addUserDebugParameter("pitch" , -np.pi/4 , np.pi/4 , 0.)
        self.yawId = p.addUserDebugParameter("yaw" , -np.pi/4 , np.pi/4 , 0.)
        self.LId = p.addUserDebugParameter("L" , -0.5 , 1.5 , 0.)
        self.LrotId = p.addUserDebugParameter("Lrot" , -1.5 , 1.5 , 0.)
        self.angleId = p.addUserDebugParameter("angleWalk" , -180. , 180. , 0.)
        self.periodId = p.addUserDebugParameter("stepPeriod" , 0.1 , 3. , 0.25)
    
    def cam_and_robotstates(self , SIM, boxId, robParm):
                ####orientacion de la camara
        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
        # p.resetDebugVisualizerCamera( cameraDistance=self.cdist*2, cameraYaw=self.cyaw/2, cameraPitch=self.cpitch*2, cameraTargetPosition=cubePos)
        keys = p.getKeyboardEvents()
        #Keys to change camera
        if keys.get(100):  #D
            self.cyaw+=1
        if keys.get(97):   #A
            self.cyaw-=1
        if keys.get(99):   #C
            self.cpitch+=1
        if keys.get(102):  #F
            self.cpitch-=1
        if keys.get(122):  #Z
            self.cdist+=.01
        if keys.get(120):  #X
            self.cdist-=.01
        if keys.get(27):  #ESC
            p.disconnect()
            sys.exit()
        #read position from debug
        start = p.readUserDebugParameter(self.start)
        if SIM:
            robParm.robotX = p.readUserDebugParameter(self.xId)
            robParm.robotY = p.readUserDebugParameter(self.yId)
            robParm.robotZ = p.readUserDebugParameter(self.zId)
            robParm.robotRoll = p.readUserDebugParameter(self.rollId)
            robParm.robotPitch = p.readUserDebugParameter(self.pitchId)
            robParm.robotYaw = p.readUserDebugParameter(self.yawId)
            robParm.gaitLength = p.readUserDebugParameter(self.LId)
            robParm.rotSpeed = p.readUserDebugParameter(self.LrotId)
            robParm.gaitYaw = p.readUserDebugParameter(self.angleId)
            robParm.T = p.readUserDebugParameter(self.periodId)
        
        return start
