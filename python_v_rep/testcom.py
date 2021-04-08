#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr  5 23:47:36 2021

@author: stav
"""

import vrep
import sys


vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
    print('CLient ID: '),
    print(clientID)
    
else:
    print ('Connection not succesful')
    sys.exit('Could not connect')
    
errorCode,rolling_jointfl=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_oneshot_wait)
errorCode,rolling_jointrl=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_oneshot_wait)

errorCode,rolling_jointfr=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_oneshot_wait)
errorCode,rolling_jointrr=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_oneshot_wait)

errorCode= vrep.simxSetJointTargetVelocity(clientID,rolling_jointfl,0.5,vrep.simx_opmode_streaming)
errorCode= vrep.simxSetJointTargetVelocity(clientID,rolling_jointrl,0.5,vrep.simx_opmode_streaming)

errorCode= vrep.simxSetJointTargetVelocity(clientID,rolling_jointfr,0.5,vrep.simx_opmode_streaming)
errorCode= vrep.simxSetJointTargetVelocity(clientID,rolling_jointrr,0.5,vrep.simx_opmode_streaming)


