# Attempt at using two cameras to detect distance. Unused.

import cv2 as cv
import imutils
import ctypes
import math
import numpy as np
import time
import serial
from playsound import playsound
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2
from mediapipe import solutions

MAX_ROTATION = 180
MAX_TILT = 14

rotation = 50
tilt = 0

flip_cam = 0

camera_dist_m = 0.03
camera_height_m = 1.06
camera_angle_dia = 55
camera_angle_hor = 48.8
camera_angle_ver = 28.7
camera_focal_length = 0.03
dist_mod = 1.5

if __name__ == '__main__':
    base_options = python.BaseOptions(model_asset_path='pose_landmarker_lite.task')
    options = vision.PoseLandmarkerOptions(base_options=base_options)
    detector = vision.PoseLandmarker.create_from_options(options)

    # Get screen info
    user32 = ctypes.windll.user32
    user32.SetProcessDPIAware()
    screensize = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)

    # Get video capture
    cap1 = cv.VideoCapture(0)
    cap1.set(3, 320)
    cap1.set(4, 240)
    cap2 = cv.VideoCapture(2)
    cap2.set(3, 320)
    cap2.set(4, 240)
    if not cap1.isOpened() or not cap2.isOpened():
        print("Could not open camera")
        exit(-1)

    while True:
        # Read frame and detect face
        success1, img1 = cap1.read()
        success2, img2 = cap2.read()
        if flip_cam == 1:
            img1 = cv.flip(img1, 0)
            img1 = cv.flip(img1, 1)
        elif flip_cam == 2:
            img2 = cv.flip(img2, 0)
            img2 = cv.flip(img2, 1)

        if img1 is not None and img2 is not None:
            img1 = imutils.resize(img1, height=int(screensize[1] * 0.8))
            mp_image1 = mp.Image(image_format=mp.ImageFormat.SRGB, data=img1)
            detection_result1 = detector.detect(mp_image1)
            pose_landmarks_list1 = detection_result1.pose_landmarks

            img2 = imutils.resize(img2, height=int(screensize[1] * 0.8))
            mp_image2 = mp.Image(image_format=mp.ImageFormat.SRGB, data=img2)
            detection_result2 = detector.detect(mp_image2)
            pose_landmarks_list2 = detection_result2.pose_landmarks

            imgCenterX, imgCenterY = int(len(img1[0]) / 2), int(len(img1) / 2)
            width, height = len(img1[0]), len(img1)

            fx1, fy1, fw1, fh1 = 0, 0, 0, 0
            fcx1, fcy1 = 0, 0
            fx2, fy2, fw2, fh2 = 0, 0, 0, 0
            fcx2, fcy2 = 0, 0
            if len(pose_landmarks_list1) > 0 and len(pose_landmarks_list2) > 0:
                pose_landmarks1 = pose_landmarks_list1[0]
                fcx1, fcy1 = int(pose_landmarks1[0].x * width), int(pose_landmarks1[0].y * height)
                img1 = cv.circle(img1, (fcx1, fcy1), 4, (0, 255, 0), 2)
                noseX1, noseY1 = int(pose_landmarks1[0].x * width), int(pose_landmarks1[0].y * height)
                pose_landmarks2 = pose_landmarks_list2[0]
                fcx2, fcy2 = int(pose_landmarks2[0].x * width), int(pose_landmarks2[0].y * height)
                img2 = cv.circle(img2, (fcx2, fcy2), 4, (0, 255, 0), 2)
                noseX2, noseY2 = int(pose_landmarks2[0].x * width), int(pose_landmarks2[0].y * height)

                # calculate distance
                #dist = (camera_dist_m * width) / (2 * math.tan(math.radians(camera_angle_hor / 2)) * (noseX1 - noseX2))
                dist = (camera_dist_m * height) / (2 * math.tan(math.radians(camera_angle_ver / 2)) * (noseY1 - noseY2)) * -1
                print(dist)

                # calculate rotation
                world_width_half = math.tan(math.radians(camera_angle_hor / 2)) / dist
                to_center_x = imgCenterX - noseX1
                world_x = world_width_half * (to_center_x / (width / 2))

                # calculate pitch
                world_height_half = math.tan(math.radians(camera_angle_ver / 2)) / dist
                to_center_y = imgCenterY - noseY1
                world_y = world_height_half * (abs(to_center_y) / (height / 2))
                if to_center_y >= 0:
                    world_y = camera_height_m + world_y
                else:
                    world_y = camera_height_m - world_y

                print(dist, world_x, world_y)
                time.sleep(1)

        # Show frame
        cv.imshow('Turret', img1)

        # Close when q is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
