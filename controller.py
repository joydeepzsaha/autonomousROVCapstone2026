import math
from pymavlink import mavutil
import time
import commands
import PID
import random
import numpy as np
import pylab as plt
import camera
import multiprocessing as mp
import cv2
 

RUN_MODE = {
    'IDLE',
    'VERTICAL',
    'FORWARD',
    'STRAFE',
}

mtx = 0
dist = 0


STATUS = {'INIT', 'SEARCH', 'APPROACH', 'ALIGN', 'ATTACH', 'DONE'}

# message_types = {'ATTITUDE', 'SCALED_IMU2', 'NAMED_VALUE_FLOAT', 'VFR_HUD'}


## Initialize robot , then do a small descend
rov = commands.Robot()
cam = camera.camera()
cam.loadCameraSettings()
rov.calibrateDepth()

# while(1):
#     rov.grabDepth()
#     time.sleep(0.5)


#PWM limits 1100-1900
xtarg = 0
yTarg = 0
zTarg = 1500

yawTarget = 90

fwdPWM = 0


x= 0; y= 0; z= 10
headingTarg = 90
rollTarg = 0
tol = 2
tf = 30
t0 = time.monotonic()
# rov.disarmRobot()



xPID = PID.PID(kp=-3e-1, ki=0, kd= -1e-5, target=xtarg)
yPID = PID.PID(kp=1e-1, ki=0, kd= 0, target=yTarg)
zPID = PID.PID(kp=1e-1, ki= 0, kd= 0, target=zTarg)
yawPID = PID.PID(kp=1.2, ki= 0, kd= 1.2, target=yawTarget, pwm_min=1420, pwm_max=1580)

u = []
ref = []
times = []
pos = []


# def directionError(current_deg, target_deg):

#     #+ve clockwise, -ve: anticlockwise
#     err = current_deg - target_deg
#     if err > 180:z
#         err -=360
#     return err

STATUS = 'INIT'

while((time.monotonic() - t0) < tf):
    match(STATUS):
        case 'INIT':
            cam.startStream()
            rov.armRobot()
            rov.setGain(0.2)
            rov.setMode('MANUAL')
            rov.lightsOff()
            pos = cam.getPos()
            if(pos is not None):
                STATUS = 'APPROACH'
            STATUS = 'SEARCH'
        case 'SEARCH':
            pos = cam.getPos()
            if(pos is not None):
                x, y, z, rot = pos
            STATUS = 'APPROACH'
        case 'APPROACH':
            pos = cam.getPos()
            if(pos is not None):
                x, y, z, rot = pos #this will be in the same units as the marker size in camera class
            angle = np.rad2deg(np.atan2(z, x))
            b = yawPID.update(angle)
            updateArrs(headingTarg, angle, b, time.monotonic()-t0)
            rov.turn(b)
            STATUS = STATUS
        case 'ALIGN':
            STATUS = 'ATTACH'
        case 'ATTACH':
            STATUS = 'DONE'
        case 'DONE':
            print('done')
            rov.disarmRobot()
# print("DisarmingRobot")
rov.disarmRobot()
cam.release()


def updateArrs(r, p, utx ,t):
    ref.append(r)
    pos.append(p)
    u.append(utx)
    times.append(t)



def updateArrs(r, p, utx ,t):
    ref.append(r)
    pos.append(p)
    u.append(utx)
    times.append(t)


plt.plot(times, ref)
plt.plot(u)
plt.plot(pos)
plt.show()
