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
        template = cv2.imread('./images/vialTop.png', 0) # * Load template    
        cv2.namedWindow("Image")
        ret, img = cam.read()
        if not ret or img.shape != (720, 1280, 3):
            img = cv2.imread('./images/opencv_frame_6_marked.png')
            offlineFlag = True
            cam.release()
        print("Press key to start, ESC to exit")
        print("#############################################")
        k = cv2.waitKey(0)
        if k%256 == 27:
            exitFunc()
        #if not offlineFlag:
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
            edges_hough = ls.HoughLinesSearch(img_binary)
            statemsg = False
            if edges_hough is not None:
                houghLocation = np.ndarray.flatten(edges_hough)
                final = ls.templatematch(img_binary, template, houghLocation)
                if final is not None:
                    statemsg = False
                    k = -1
                    grabPoints, grabAngle = ls.grabberPoint(houghLocation)
                    grabPoints = np.around(grabPoints)
                    grabPoints = grabPoints.astype(int)
                    cv2.circle(final, (grabPoints[0], grabPoints[1]), 3, color = (0,255,0), thickness=2)
                    cv2.circle(final, (grabPoints[2], grabPoints[3]), 3, color = (0,255,0), thickness=2)
                    cv2.circle(final, (grabPoints[4], grabPoints[5]), 3, color = (0,0,255), thickness=2)
                    print("Processing time: %s s" % (time.time() - start_time))
                    print("#############################################")
                    cv2.imshow('Detection', final)
                    print("Press space to initiate pickup")
                    print("Press ESC to exit")
                    print("Press other to retry")
                    print("#############################################")
                    x,y = ls.pixelstocm([grabPoints[4], grabPoints[5]], final.shape)
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
