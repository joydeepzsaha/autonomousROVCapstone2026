import math
from pymavlink import mavutil
import time
import commands

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


# Initialize robot , then do a small descend
rov = commands.Robot()
rov.calibrateDepth()

# while(1):
#     rov.grabDepth()
#     time.sleep(1)

#PWM limits 1100-1900

STATUS = 'INIT'
match(STATUS):
    case 'INIT':
        mtx, dist = commands.loadCameraSettings()
        rov.armRobot()
        rov.setGain(0.3)
        rov.setMode('MANUAL')
        rov.lightsOn()
        if(commands.poseEstimate is not None):
            STATUS = 'ALIGN'
        STATUS = 'SEARCH'
    case 'SEARCH':
        STATUS = 'APPROACH'
    case 'APPROACH':
        STATUS = 'ALIGN'
    case 'ALIGN':
        STATUS = 'ATTACH'
    case 'ATTACH':
        STATUS = 'DONE'
    case 'DONE':
        rov.disarmRobot()

