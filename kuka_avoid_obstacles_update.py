#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Apr 11 11:27:02 2021

@author: stav
"""


#Import Libraries:
import vrep                  #V-rep library
import sys
import time                #used to keep track of time
import numpy as np         #array library
import math


#Pre-Allocation

PI=math.pi  #pi=3.14..., constant

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:  #check if client connection successful
    print ('Connected to remote API server')
    
else:
    print ('Connection not successful')
    sys.exit('Could not connect')


#retrieve rolling joints

errorCode,rolling_jointfl=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_oneshot_wait)
errorCode,rolling_jointrl=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_oneshot_wait)

errorCode,rolling_jointfr=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_oneshot_wait)
errorCode,rolling_jointrr=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_oneshot_wait)


sensor_h=[] #empty list to store all the handles for the proximity sensor
sensor_val=np.array([]) #empty array for sensor measurements

#orientation of all the sensors relative to the centre of the robot 
sensor_loc=np.array([-PI/2, -50/180.0*PI,-30/180.0*PI,-10/180.0*PI,10/180.0*PI,30/180.0*PI])

#for loop to retrieve sensor arrays and initiate sensors
for x in range(1,6+1):
        errorCode,sensor_handle=vrep.simxGetObjectHandle(clientID,'kuka_sensor'+str(x),vrep.simx_opmode_oneshot_wait)
        sensor_h.append(sensor_handle) #keep list of handles        
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_handle,vrep.simx_opmode_streaming) #reading of proximity sensor          
        sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) #get list of values, using linear algebra to turn the number from 3-dimensional to 1-dimensional
        

t = time.time() 

#while loop for the script to run for 60 seconds
while (time.time()-t)<60:
    #Loop Execution
    sensor_val=np.array([])    
    for x in range(1,6+1):
        #read the sensor values and store them in the sensor_val
        errorCode,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,sensor_h[x-1],vrep.simx_opmode_buffer)                
        sensor_val=np.append(sensor_val,np.linalg.norm(detectedPoint)) #get list of values

    
    #controller specific
    sensor_sq=sensor_val[0:4]*sensor_val[0:4] #square the values of front-facing sensors
        
    min_ind=np.where(sensor_sq==np.min(sensor_sq)) #find minimum sensor value, where the obstacle is more close
    min_ind=min_ind[0][0] #turn array to a simple number
    
    if sensor_sq[min_ind]<0.2: #check if minimum values is less that 0.2
        steer=-1/sensor_loc[min_ind] #find the sensor that is closest to the robot
    else:
        steer=0
            
    
    v=1	#forward velocity
    kp=0.5	#steering gain
    vl=v+kp*steer #left velocity
    vr=v-kp*steer #right velocity
    print ("V_l =",vl) #print out the values
    print ("V_r =",vr)

    # send the target velocity to the robot
    errorCode=vrep.simxSetJointTargetVelocity(clientID,rolling_jointfl,vl, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,rolling_jointrl,vl,vrep.simx_opmode_streaming)
    
    errorCode=vrep.simxSetJointTargetVelocity(clientID,rolling_jointfr,vr, vrep.simx_opmode_streaming)
    errorCode=vrep.simxSetJointTargetVelocity(clientID,rolling_jointrr,vr, vrep.simx_opmode_streaming)
    
    

    time.sleep(0.2) #loop executes once every 0.2 seconds (= 5 Hz)

#Post ALlocation, after 60 seconds the robot stops (velocity=0)
    
errorCode= vrep.simxSetJointTargetVelocity(clientID,rolling_jointfl,0,vrep.simx_opmode_streaming)
errorCode= vrep.simxSetJointTargetVelocity(clientID,rolling_jointrl,0,vrep.simx_opmode_streaming)

errorCode= vrep.simxSetJointTargetVelocity(clientID,rolling_jointfr,0,vrep.simx_opmode_streaming)
errorCode= vrep.simxSetJointTargetVelocity(clientID,rolling_jointrr,0,vrep.simx_opmode_streaming)
