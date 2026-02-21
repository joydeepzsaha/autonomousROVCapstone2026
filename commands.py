import math
from pymavlink import mavutil
import time
import numpy as np
import cv2


#each RC Channel corresponds to a different DOF

channel = {
    "pitch": 1,
    "roll" : 2,
    "vertical": 3,
    "yaw": 4,
    "forward": 5,
    "lateral": 6, 
    "pan": 7,
    "tilt": 8,
    "lights1": 9,
    "lights2": 10
}

def initConnection():
    protocol = 'tcp'; address = '192.168.2.2'; port = '6777'; connection = f'{protocol}:{address}:{port}'

    #Create connection from topside
    connection = mavutil.mavlink_connection(connection)

    #verify connection
    connection.wait_heartbeat()
    return connection

class Robot:
    def __init__(self):
        self.robot = initConnection()
        pass


#BASIC/NECESSARY COMMANDS

    def armRobot(self):
        print("Arming robot")
        self.robot.arducopter_arm()
        self.robot.motors_armed_wait()
        print("Robot armed")

    #STABILIZE, MANUAL, DEPTH HOLD

    def setMode(self, mode):
        if mode not in self.robot.mode_mapping():
            print('Error, unknown mode')
            self.robot.disarmRobot()
        else:
            mode_id = self.robot.mode_mapping()[mode]
            self.robot.set_mode(mode_id)

    def disarmRobot(self):
        print("Disarming Robot")
        self.robot.arducopter_disarm()

    def setGain(self, gain=0.3):
        param = b'PILOT_GAIN'
        self.robot.mav.param_set_send(
            self.robot.target_system,
            self.robot.target_component,
            param,
            gain,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def calibrateDepth(self):
        self.robot.mav.command_long_send(
        self.robot.target_system,
        self.robot.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0, 0, 1, 0, 0, 0, 0, 0
    )


    def grabDepth(self):
        self.robot.mav.command_long_send(
        self.robot.target_system,
        self.robot.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        137,
        0, 0, 0, 0, 0, 0
    )
        msg = self.robot.recv_match(type='SCALED_PRESSURE2', blocking=True)
        if msg:
            pressure = msg.press_abs
            print(pressure)
            depth = (pressure - 1013.25) / 98.0665
            # print(f"Depth: {depth:.3f} m")
            return depth
        else:
            print("No response received")
            return None
        

    ##-----------------MOTION CONTROL----------------------
    ##-----------------------------------------------------

    def set_rc_channel_pwm(self, channel_id, pwm=1500):
        """ Set RC channel pwm value
        Args:
            channel_id (TYPE): Channel ID
            pwm (int, optional): Channel pwm value 1100-1900
        """
        if channel_id < 1 or channel_id > 18:
            print("Channel does not exist.")
            return

        # Mavlink 2 supports up to 18 channels:
        # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
        rc_channel_values = [65535 for _ in range(18)]
        rc_channel_values[channel_id - 1] = pwm
        self.robot.mav.rc_channels_override_send(
            self.robot.target_system,                # target_system
            self.robot.target_component,             # target_component
            *rc_channel_values)  


    def stopMotion(self):
        for i in range(1, 6):
            self.set_rc_channel_pwm(i, 1500)

    def goForward(self, pwm):
        id = channel.get("forward")
        self.set_rc_channel_pwm(id, pwm)

    def goVertical(self, pwm):
        id = channel.get("vertical")
        self.set_rc_channel_pwm(id, pwm)

    def turn(self, pwm):
        id = channel.get("turn")
        self.set_rc_channel_pwm(id, pwm)

    def roll(self, pwm):
        id = channel.get("roll")
        self.set_rc_channel_pwm(id, pwm)

    def strafe(self, pwm):
        id = channel.get("lateral")
        self.set_rc_channel_pwm(id, pwm)

    ##-----------------CAMERA CONTROL----------------------
    ##-----------------------------------------------------


    def tilt2Angle(tilt):
        print(tilt)
        tilt = tilt - 0.3
        return tilt * 100 #deg

    def getCameraTilt(self):
        self.robot.mav.command_long_send(
        self.robot.target_system,
        self.robot.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        251, 0, 0, 0, 0, 0, 0
    )
        msg = self.robot.recv_match(type="NAMED_VALUE_FLOAT")
        if msg:
            name = msg.name.decode('utf-8') if isinstance(msg.name, bytes) else msg.name
            value = msg.value
            if name == 'CamTilt':
                print(value)
                return value
            else: 
                return self.getCameraTilt()
        else:
            return self.getCameraTilt()

    def lights(self, pwm):
        id = channel.get("lights1")
        self.set_rc_channel_pwm(id, pwm)

    def lightsOn(self):
        self.lights(1900)

    def lightsOff(self):
        self.lights(1100)


    ##-----------------OPENCV COMMANDS---------------------
    ##-----------------------------------------------------

def loadCameraSettings():
    calibration_data = np.load('camera_calibration.npz')

    # Access the individual arrays
    mtx = calibration_data['mtx']
    dist = calibration_data['dist'] 
    return mtx, dist

##Borrowed from stack overflow
##https://stackoverflow.com/questions/76802576/how-to-estimate-pose-of-single-marker-in-opencv-python-4-8-0
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
    corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                            [marker_size / 2, marker_size / 2, 0],
                            [marker_size / 2, -marker_size / 2, 0],
                            [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash


def poseEstimate(mtx, dist, img):
    dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36H11)
    params = cv2.aruco.DetectorParameters()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detector = cv2.aruco.ArucoDetector(dict, params)

    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        rvec, tvec, _ = my_estimatePoseSingleMarkers(corners, 0.1, mtx, dist)
        #time.sleep(1e-4) #can get rid of this maybe
        return rvec, tvec
    return None

def rvecToMat(rvec):
    matrix, _ = cv2.Rodrigues(rvec)
    return matrix

def createTransformationMatrix(rvec, tvec):
    mat = rvecToMat(rvec)
    return np.hstack((mat, tvec))

def getPos():
    rvec, tvec = poseEstimate()
    mat = createTransformationMatrix(rvec, tvec)
    mat = np.linalg.inv(mat)


