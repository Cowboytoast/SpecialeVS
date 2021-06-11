import cv2
import time
import sys
import math
sys.path.append("./images")
from cv2 import aruco

import LineSearchLib as ls
import PreprocessingLib as prep
import RobotLib as rl
import CalibrationLib as cb
import numpy as np

#*********** GLOBAL PARAMETERS **************
state = "init"
statemsg = False

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
        var_exists = False
    else:
        cam.release()

    try:
        s
    except NameError:
        var_exists = False
    else:
        s.close()

    try:
        thread
    except NameError:
        var_exists = False
    else:
        thread.shutdown()

    cv2.destroyAllWindows()
    exit()

#**********************Main loop********************************
while True:
    if state == "init":
        angleTolerance = 0.3
        extractCounter = 0
        handOffPos = rl.handOffPosLOT()
        global cam
        cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        offlineFlag = False
        start_time = time.time()
        template = cv2.imread('./images/VialTop.png', 0) # * Load template
        cv2.namedWindow("Image")
        ret, img = cam.read()
        if not ret or img.shape != (720, 1280, 3):
            #! TODO SOON:
            #! Speed up robot
            #! Test special cases
            
            #! Error on 0: Short line
            #! Error on 12: One line too long
            img = cv2.imread('./final_images/final_setup_0.png')
            offlineFlag = True
            cam.release()
        print("Press key to start, ESC to exit")
        print("#############################################")
        k = cv2.waitKey(0)
        if k%256 == 27:
            exitFunc()
        if not offlineFlag:
            rl.robotInit()
        state = "sourceimg"

    if state == "sourceimg":
        if statemsg == False:
            print("Press c to calibrate")
            print("Press other to process image")
            print("Press ESC to exit")
            print("#############################################")
            statemsg = True
        if not offlineFlag:
            ret, img = cam.read()
        cv2.imshow("Image", img)
        k = cv2.waitKey(5)
        if k%256 == 27:
            exitFunc()
        if k%256 == 99:
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

        elif k != -1:
            cv2.imwrite('unproc.png', img)
            start_time = time.time()
            edges_hough = None
            img_binary = prep.PrepImg(img)
            cv2.imwrite('procced.png', img_binary)
            cv2.imshow("Binary image", img_binary)
            cv2.waitKey(5)
            houghLocation = ls.HoughLinesSearch(img_binary)
            statemsg = False
            #! DELETE BELOW, ONLY FOR PIC 0 TESTING
            #houghLocation = np.array([[50, 277, 244, 278, 117, 115]])
            if houghLocation is not None and houghLocation.size > 0:
                houghLocation = ls.removeExtras(houghLocation) # Removes superfluous lines
                houghLocation = ls.LineExtend(img_binary, houghLocation)
                houghLocation = np.ndarray.flatten(houghLocation)
                final, UpDown, grabPoint, grabAngle = ls.templatematch(img_binary, template, houghLocation)
                if final is not None:
                    statemsg = False
                    k = -1
                    #grabPoints, grabAngle = ls.grabberPoint(houghLocation, UpDown)
                    grabPoints = np.around(grabPoint)
                    grabPoints = grabPoint.astype(int)
                    #cv2.circle(final, (grabPoints[0], grabPoints[1]), 3, color = (0,255,0), thickness=2)
                    #cv2.circle(final, (grabPoints[2], grabPoints[3]), 3, color = (0,255,0), thickness=2)
                    print("Processing time: %s s" % (time.time() - start_time))
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
        print('Initiating pickup at (x,y) = (%.3f,%.3f) cm' % (x, y))
        print('Angle of %f deg' %(math.degrees(grabAngle)))
        print("#############################################")
        rl.robotRun(x, y, rz = grabAngle)
        state = "sourceimg"
