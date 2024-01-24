import cv2 as cv
import imutils
import ctypes
import math
import time
import serial
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

CAM_IDX = 1

buffer = 100
currentPitch = 100
currentYaw = 50
pitchSpeedMin = 1.0
pitchSpeedMax = 8.0
pitchSpeed = 1.0
yawSpeedMin = 0.1
yawSpeedMax = 2.5
yawSpeed = 1
earDistMin = 25.0
earDistMax = 1000.0
delay = 0.1
maxNoDetection = 2
shootCooldown = 2
scanSpeed = 0.5


def yaw(ser, amount):
    global currentYaw
    currentYaw += amount
    if currentYaw < 0:
        currentYaw = 0
    elif currentYaw > 100:
        currentYaw = 100
    ser.write(bytes("rotate " + str(int(currentYaw)) + "\n", 'utf-8'))


def pitch(ser, amount):
    global currentPitch
    currentPitch += amount
    if currentPitch < 0:
        currentPitch = 0
    elif currentPitch > 100:
        currentPitch = 100
    ser.write(bytes("tilt " + str(int(currentPitch)) + "\n", 'utf-8'))


def fire(ser):
    ser.write(bytes("fire 0", 'utf-8'))


if __name__ == '__main__':
    # Setup serial connection
    arduino = serial.Serial(port='COM9', baudrate=115200, timeout=.01)

    # Move to start position
    arduino.write(bytes("rotate " + str(int(currentYaw)) + "\n", 'utf-8'))
    time.sleep(0.1)
    while arduino.in_waiting == 0:
        continue
    arduino.write(bytes("tilt  " + str(int(currentPitch)) + "\n", 'utf-8'))

    # Setup pose detection model
    base_options = python.BaseOptions(model_asset_path='pose_landmarker_lite.task')
    options = vision.PoseLandmarkerOptions(base_options=base_options)
    detector = vision.PoseLandmarker.create_from_options(options)

    # Get screen info
    user32 = ctypes.windll.user32
    user32.SetProcessDPIAware()
    screensize = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)

    # Get video capture
    cap = cv.VideoCapture(CAM_IDX)
    if not cap.isOpened():
        print("Could not open camera")
        exit(-1)

    detected = time.time()
    delayTimer = time.time()
    shootTimer = time.time()
    while True:
        if time.time() - delayTimer >= 2:
            delayTimer = time.time()

        # Read frame and detect pose
        success, img = cap.read()
        img = imutils.resize(img, height=int(screensize[1] * 0.8))
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img)
        detection_result = detector.detect(mp_image)
        pose_landmarks_list = detection_result.pose_landmarks

        imgCenterX, imgCenterY = int(len(img[0]) / 2), int(len(img) / 2)
        width, height = len(img[0]), len(img)

        fx, fy, fw, fh = 0, 0, 0, 0
        fcx, fcy = 0, 0
        if len(pose_landmarks_list) > 0:
            detected = time.time()
            # Get detected position
            pose_landmarks = pose_landmarks_list[0]
            fcx, fcy = int(pose_landmarks[0].x * width), int(pose_landmarks[0].y * height)
            img = cv.circle(img, (fcx, fcy), 4, (0, 255, 0), 2)
            leftEarX, leftEarY = int(pose_landmarks[7].x * width), int(pose_landmarks[7].y * height)
            rightEarX, rightEarY = int(pose_landmarks[8].x * width), int(pose_landmarks[8].y * height)
            earDist = math.sqrt(math.pow(leftEarX - rightEarX, 2) + math.pow(leftEarY - rightEarY, 2))

            # Calculate yaw and pitch speed according to estimated distance (bigger distance between ears -> closer to camera)
            ratio = math.log(earDist, 2) / math.log(earDistMax, 2)
            buffer = earDist
            yawSpeed = (yawSpeedMax - yawSpeedMin) * ratio + yawSpeedMin
            pitchSpeed = (pitchSpeedMax - pitchSpeedMin) * ratio + pitchSpeedMin

            # Calculate where how to move to target
            if time.time() - delayTimer >= delay and time.time() - detected < maxNoDetection:
                delayTimer = time.time()
                # Calculate where to move
                doYaw = 0
                doPitch = 0
                if fcx < imgCenterX - buffer:
                    doYaw = 1
                elif fcx > imgCenterX + buffer:
                    doYaw = -1
                if fcy < imgCenterY - buffer:
                    doPitch = 1
                elif fcy > imgCenterY + buffer:
                    doPitch = -1

                # Shoot if target in range
                if doYaw == 0 and time.time() - shootTimer > shootCooldown:
                    fire(arduino)
                    shootTimer = time.time()

                # Move to target
                if doPitch != 0:
                    pitch(arduino, doPitch * pitchSpeed)
                if doYaw != 0:
                    yaw(arduino, doYaw * yawSpeed)

        # If no target in frame for certain amount of time, start scanning
        # fps drops to below 0 after a while when this is active for some reason
        """if time.time() - detected >= maxNoDetection:
            yaw(arduino, scanSpeed)
            if currentYaw >= 100:
                scanSpeed *= -1
                pitch(arduino, 100)
            elif currentYaw <= 0:
                scanSpeed *= -1
                pitch(arduino, -100)"""

        # Show frame
        cv.imshow('Turret', img)

        # Close when q is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
