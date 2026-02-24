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


STATUS = 'INIT'

while((time.monotonic() - t0) < tf):
    # k = cv2.waitKey(1)
    # if k%256 == 27:
    #     # ESC pressed
    #     print("Escape hit, closing...")
    #     break
# while(1):
    match(STATUS):
        case 'INIT':
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
                # x = x*1e3; y =y*1e3; z=z*1e3
            #     xtarg = x+1
            #     yTarg = y+1
            #     zTarg = z+1
            #     print('ztarg', zTarg)

                # xPID.updateTarget(xtarg); yPID.updateTarget(yTarg); zPID.updateTarget(zTarg)
            STATUS = 'APPROACH'
        case 'APPROACH':
            # print("APP")
            pos = cam.getPos()
            if(pos is not None):
                x, y, z, rot = pos
                # x = x*1e3; y =y*1e3; z=z*1e3
                # print("z", z)
            print("angle:", np.rad2deg(np.atan2(z, x)))
            # print("X and Z:", x, z)
            rov.turn(yawPID.update(np.rad2deg(np.atan2(z, x))));# rov.goVertical(yPID.update(y)); rov.goForward(zPID.update(z))
            # rov.goForward(zPID.update(z))
                # time.sleep(1e-1)
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




