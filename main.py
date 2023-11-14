import cv2 as cv
import imutils
import ctypes
from playsound import playsound
import time

playSounds = True
buffer = 20
maxTargetLostTime = 2

if __name__ == '__main__':
    # Get screen info
    user32 = ctypes.windll.user32
    user32.SetProcessDPIAware()
    screensize = user32.GetSystemMetrics(0), user32.GetSystemMetrics(1)

    # Get video capture
    cap = cv.VideoCapture(0)  # Change to 0 to use the built-in camera
    if not cap.isOpened():
        print("Could not open camera")
        exit(-1)

    # Get the facial detection cascade
    faceCascade = cv.CascadeClassifier(cv.data.haarcascades + "haarcascade_frontalface_alt.xml")

    targetSpotted = False
    targetAcquired = False
    targetLostTimer = 0
    start = time.time()
    while True:
        if time.time() - start >= 2:
            start = time.time()
            if playSounds:
                playsound('sounds/radar.wav')

        # Read frame and detect face
        success, img = cap.read()
        img = cv.flip(img, 1)
        img = imutils.resize(img, height=int(screensize[1] * 0.8))
        imgGrey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        imgGrey = cv.equalizeHist(imgGrey)
        faces = faceCascade.detectMultiScale(imgGrey, 1.3, 5)

        imgCenter = [int(len(img[0]) / 2), int(len(img) / 2)]

        # Draw reticle
        img = cv.circle(img, (imgCenter[0], imgCenter[1]), 60, (0, 0, 255), 2)
        img = cv.line(img, (imgCenter[0], imgCenter[1] - 60), (imgCenter[0], imgCenter[1] - 10), (0, 0, 255), 2)
        img = cv.line(img, (imgCenter[0], imgCenter[1] + 60), (imgCenter[0], imgCenter[1] + 10), (0, 0, 255), 2)
        img = cv.line(img, (imgCenter[0] - 60, imgCenter[1]), (imgCenter[0] - 15, imgCenter[1]), (0, 0, 255), 2)
        img = cv.line(img, (imgCenter[0] + 60, imgCenter[1]), (imgCenter[0] + 15, imgCenter[1]), (0, 0, 255), 2)
        img = cv.line(img, (imgCenter[0], imgCenter[1] + 15), (imgCenter[0], imgCenter[1] - 15), (0, 0, 255), 1)
        img = cv.line(img, (imgCenter[0] + 15, imgCenter[1]), (imgCenter[0] - 15, imgCenter[1]), (0, 0, 255), 1)

        if len(faces) > 0:
            if not targetSpotted:
                targetSpotted = True
                targetLostTimer = time.time()
                if playSounds:
                    playsound('sounds/target_spotted.wav')

            # Draw rectangle around face
            fx, fy, fw, fh = faces[0]
            fcx, fcy = fx + int(fw / 2), fy + int(fh / 2)
            img = cv.rectangle(img, (fx, fy), (fx + fw, fy + fh), (0, 255, 0), 2)
            img = cv.circle(img, (fcx, fcy), 2, (0, 255, 0), 2)

            # Calculate where to move
            yaw = False
            pitch = False
            if fcx < imgCenter[0] - buffer:
                print("yaw right", end="")
                yaw = True
            elif fcx > imgCenter[0] + buffer:
                print("yaw left", end="")
                yaw = True
            if fcy < imgCenter[1] - buffer:
                if yaw:
                    print(" and ", end="")
                print("pitch up", end="")
                pitch = True
            elif fcy > imgCenter[1] + buffer:
                if yaw:
                    print(" and ", end="")
                print("pitch down", end="")
                pitch = True
            print("")

            if not pitch and not yaw:
                if not targetAcquired:
                    targetAcquired = True
                    if playSounds:
                        playsound('sounds/target_acquired.wav')
            else:
                if targetAcquired:
                    targetAcquired = False

        else:
            if targetSpotted:
                if time.time() - targetLostTimer >= maxTargetLostTime:
                    targetSpotted = False
                    if playSounds:
                        playsound('sounds/target_lost.wav')

        # Show frame
        cv.imshow('Turret', img)

        # Close when q is pressed
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()
