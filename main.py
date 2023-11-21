import cv2 as cv
import imutils
import ctypes
from playsound import playsound
import time
import serial

playSounds = True
buffer = 100
maxTargetLostTime = 2

currentPitch = 0
currentYaw = 50
pitchSpeed = 20
yawSpeed = 2
delay = 0.1
maxNoDetection = 2
shootCooldown = 2


def yaw(ser, amount):
    global currentYaw
    currentYaw += amount
    if currentYaw < 0:
        currentYaw = 0
    elif currentYaw > 100:
        currentYaw = 100
    else:
        ser.write(bytes("rotate " + str(currentYaw) + "\n", 'utf-8'))


def pitch(ser, amount):
    global currentPitch
    currentPitch += amount
    if currentPitch < 0:
        currentPitch = 0
    elif currentPitch > 100:
        currentPitch = 100
    else:
        ser.write(bytes("tilt " + str(currentPitch) + "\n", 'utf-8'))


def fire(ser):
    ser.write(bytes("fire 0", 'utf-8'))


if __name__ == '__main__':
    arduino = serial.Serial(port='COM9', baudrate=115200, timeout=.01)
    arduino.write(bytes("rotate 50\n", 'utf-8'))
    time.sleep(0.1)
    while arduino.in_waiting == 0:
        continue
    arduino.write(bytes("tilt 0\n", 'utf-8'))

    # Get screen info
    user32 = ctypes.windll.user32
    user32.SetProcessDPIAware()
    screensize = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)

    # Get video capture
    cap = cv.VideoCapture(1)  # Change to 0 to use the built-in camera
    if not cap.isOpened():
        print("Could not open camera")
        exit(-1)

    # Get the facial detection cascade
    faceCascade = cv.CascadeClassifier(cv.data.haarcascades + "haarcascade_frontalface_alt.xml")

    detected = time.time()
    targetSpotted = False
    targetAcquired = False
    targetLostTimer = 0
    delayTimer = time.time()
    shootTimer = time.time()
    while True:
        if time.time() - delayTimer >= 2:
            delayTimer = time.time()
            if playSounds:
                playsound('sounds/radar.wav')

        # Read frame and detect face
        success, img = cap.read()
        #img = cv.flip(img, 1)
        img = imutils.resize(img, height=int(screensize[1] * 0.8))
        imgGrey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        imgGrey = cv.equalizeHist(imgGrey)
        faces = faceCascade.detectMultiScale(imgGrey, 1.3, 5)

        imgCenter = [int(len(img[0]) / 2), int(len(img) / 2)]

        # Draw reticle
        """img = cv.circle(img, (imgCenter[0], imgCenter[1]), 60, (0, 0, 255), 2)
        img = cv.line(img, (imgCenter[0], imgCenter[1] - 60), (imgCenter[0], imgCenter[1] - 10), (0, 0, 255), 2)
        img = cv.line(img, (imgCenter[0], imgCenter[1] + 60), (imgCenter[0], imgCenter[1] + 10), (0, 0, 255), 2)
        img = cv.line(img, (imgCenter[0] - 60, imgCenter[1]), (imgCenter[0] - 15, imgCenter[1]), (0, 0, 255), 2)
        img = cv.line(img, (imgCenter[0] + 60, imgCenter[1]), (imgCenter[0] + 15, imgCenter[1]), (0, 0, 255), 2)
        img = cv.line(img, (imgCenter[0], imgCenter[1] + 15), (imgCenter[0], imgCenter[1] - 15), (0, 0, 255), 1)
        img = cv.line(img, (imgCenter[0] + 15, imgCenter[1]), (imgCenter[0] - 15, imgCenter[1]), (0, 0, 255), 1)"""

        fx, fy, fw, fh = 0, 0, 0, 0
        fcx, fcy = 0, 0
        if len(faces) > 0:
            detected = time.time()
            # Draw rectangle around face
            fx, fy, fw, fh = faces[0]
            fcx, fcy = fx + int(fw / 2), fy + int(fh / 2)
            img = cv.rectangle(img, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 2)
            img = cv.circle(img, (fcx, fcy), 2, (0, 255, 0), 2)

            img = cv.line(img, (fcx, imgCenter[1]), (fcx, fcy), (0, 0, 255) if abs(fcy - imgCenter[1]) > buffer else (255, 0, 0), 2)
            img = cv.line(img, (imgCenter[0], fcy), (fcx, fcy), (0, 0, 255) if abs(fcx - imgCenter[0]) > buffer else (255, 0, 0), 2)

            if not targetSpotted:
                targetSpotted = True
                targetLostTimer = time.time()
                if playSounds:
                    playsound('sounds/target_spotted.wav')
        else:
            if targetSpotted:
                if time.time() - targetLostTimer >= maxTargetLostTime:
                    targetSpotted = False
                    if playSounds:
                        playsound('sounds/target_lost.wav')

        if time.time() - delayTimer >= delay and time.time() - detected < maxNoDetection:
            delayTimer = time.time()
            # Calculate where to move
            doYaw = 0
            doPitch = 0
            if fcx < imgCenter[0] - buffer:
                print("yaw right", end="")
                doYaw = 1
            elif fcx > imgCenter[0] + buffer:
                print("yaw left", end="")
                doYaw = -1
            if fcy < imgCenter[1] - buffer:
                if doYaw != 0:
                    print(" and ", end="")
                print("pitch up", end="")
                doPitch = 1
            elif fcy > imgCenter[1] + buffer:
                if doYaw != 0:
                    print(" and ", end="")
                print("pitch down", end="")
                doPitch = -1
            print("")

            if doPitch == 0 and doYaw == 0:
                if not targetAcquired:
                    targetAcquired = True
                    if playSounds:
                        playsound('sounds/target_acquired.wav')
                if time.time() - shootTimer > shootCooldown:
                    fire(arduino)
                    shootTimer = time.time()
            else:
                if targetAcquired:
                    targetAcquired = False
                if doPitch != 0:
                    pitch(arduino, doPitch * pitchSpeed)
                if doYaw != 0:
                    yaw(arduino, doYaw * yawSpeed)

        # Show frame
        cv.imshow('Turret', img)

        # Close when q is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()
