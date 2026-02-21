import math
from pymavlink import mavutil
import time
import commands
import PID
import random
import numpy as np
import pylab as plt
 

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
# rov.calibrateDepth()

while(1):
    rov.grabDepth()
    time.sleep(0.5)

#PWM limits 1100-1900
zTarg = -1
xtarg = 0
yTarg = 0
headingTarg = 0
rollTarg = 0
tol = 2
tf = 2
t0 = time.monotonic()

vertPID = PID.PID(kp=1e-2, ki=0, kd= 0, target=0)
STATUS = 'INIT'

# while(time.monotonic() - t0 < tf):
#     match(STATUS):
#         case 'INIT':
#             mtx, dist = commands.loadCameraSettings()
#             rov.armRobot()
#             rov.setGain(0.3)
#             rov.setMode('MANUAL')
#             rov.lightsOn()
#             vertPID.update_target(zTarg)
#             if(commands.poseEstimate is not None):
#                 STATUS = 'ALIGN'
#             STATUS = 'SEARCH'
#         case 'SEARCH':
#             meas = rov.grabDepth()
#             vertPWM = vertPID.update(meas)
#             rov.goVertical(vertPWM)
#             if(vertPID.atTarget(meas)):
#                 STATUS = 'DONE'
#             STATUS = STATUS
#         case 'APPROACH':
#             STATUS = 'ALIGN'
#         case 'ALIGN':
#             STATUS = 'ATTACH'
#         case 'ATTACH':
#             STATUS = 'DONE'
#         case 'DONE':
#             print('done')
#             rov.disarmRobot()




