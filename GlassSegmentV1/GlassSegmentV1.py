import cv2
import time
import math

import LineSearchLib as ls
import PreprocessingLib as prep
import RobotLib as rl
import CalibrationLib as cb
import numpy as np

#*********** GLOBAL PARAMETERS **************
state = "init"
statemsg = False
corners = 0
#********************************************
def exitFunc():
    global cam
    global s
    global thread
    print("Escape hit, closing...")
    print("#############################################")

    try:
        cam
    except NameError:
        pass
    else:
        cam.release()

    try:
        s
    except NameError:
        pass
    else:
        s.close()

    try:
        thread
    except NameError:
        pass
    else:
        thread.shutdown()

    cv2.destroyAllWindows()
    exit()

#**********************Main loop********************************
while True:
    if state == "init":
        # Sets initial variables and initializes robot connection
        extractCounter = 0
        handOffPos = rl.handOffPosLOT()
        global cam
        cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        offlineFlag = False
        start_time = time.time()
        template = cv2.imread('./templates/VialTop.png', 0) # * Load template
        cv2.namedWindow("Image")
        ret, img = cam.read()
        if not ret or img.shape != (720, 1280, 3):
            img = cv2.imread('./final_images/final_setup_6.png')
            offlineFlag = True
            cam.release()
        print("Press key to start, ESC to exit")
        print("#############################################")
        k = cv2.waitKey(0)
        if k%256 == 27:
            exitFunc()
        if not offlineFlag:
            rl.robotInit()
        state = "getimg"

    if state == "getimg":
        # Captures images/calibrates/moves the robot to start position
        if statemsg == False:
            print("Press c to calibrate")
            print("Press s to go to start position")
            print("Press other to process image")
            print("Press ESC to exit")
            print("#############################################")
            statemsg = True
        # Checks if the PC is connected to the camera
        if not offlineFlag:
            ret, img = cam.read()
        cv2.imshow("Image", img)
        k = cv2.waitKey(5)
        if k%256 == 27: # 'ESC' pressed
            exitFunc()
        elif k%256 == 99: # 'c' pressed
            k = -1
            corners = cb.markerCalib(img)
            if corners is not None:
                print("Calibration done")
                print("#############################################")
                cv2.waitKey(5)
                statemsg = False
            else:
                print("Not enough markers, try again")
                print("#############################################")
                statemsg = False
        elif k%256 == 115: # 's' pressed
            print("Going to start pos.")
            print("#############################################")
            statemsg = False
            rl.startPos()
        # Processes image
        elif k != -1 and corners is not None:
            edges_hough = None
            img_cropped, img_binary = prep.PrepImg(img, corners)
            print("Preprocessing time: %s s" % (time.time() - start_time))
            print("#############################################")
            cv2.imshow("Binary image", img_binary)
            cv2.waitKey(5)
            houghMerged, houghAll, houghLocation = ls.HoughLinesSearch(img_binary)
            statemsg = False
            if houghLocation is not None and houghLocation.size > 0:
                houghLocation = ls.removeExtras(houghLocation) # Removes superfluous lines
                houghLocation = ls.LineExtend(img_binary, houghLocation)
                houghLocation = np.ndarray.flatten(houghLocation)
                final, UpDown, grabPoint, grabAngle = ls.templatematch(img_binary, template, houghLocation)
                if final is not None:
                    statemsg = False
                    k = -1
                    print("#############################################")
                    cv2.imshow('Detection', final)
                    print("Press space to initiate pickup")
                    print("Press ESC to exit")
                    print("Press other to retry")
                    print("#############################################")
                    x,y = ls.pixelstocm([grabPoint[0], grabPoint[1]], final.shape)
                    k = cv2.waitKey(0)
                    if k%256 == 27:
                        exitFunc()
                    if k%256 == 32:
                        state = "pickup"
            else:
                print("Not enough lines found")
                print("#############################################")
                statemsg = False

    if state == "pickup":
        print('Initiating pickup at (x,y) = (%.1f,%.1f) cm' % (x * 100, y * 100))
        print('Angle of %f deg' %(math.degrees(grabAngle)))
        print("#############################################")
        rl.robotRunV2(x, y, rz = grabAngle)
        state = "getimg"
