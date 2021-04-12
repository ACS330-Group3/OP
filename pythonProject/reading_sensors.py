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
    print(f'CLient ID: {clientID:d}')
    
else:
    print ('Connection not succesful')
    sys.exit('Could not connect')
    

#to read sensor1 position 
errorCode,sensor1=vrep.simxGetObjectHandle(clientID,'kuka_sensor1',vrep.simx_opmode_oneshot_wait)
errorCode,detectionStatearray, detectedPoint, detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor1,vrep.simx_opmode_streaming)


