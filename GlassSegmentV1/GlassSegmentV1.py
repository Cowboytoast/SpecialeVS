import cv2
import time
import sys
sys.path.append("./images")
from cv2 import aruco

import LineSearchLib as ls
import PreprocessingLib as prep
import RobotLib as rl
import CalibrationLib as cb
import numpy as np
from scipy import ndimage
# TODO: Fix line merge. Der sker en fejl ved billede opencv_frame_1
# TODO: Lav movement filer om således at vi har et 'init' call og kan kalde de andre filer med argumenter for position.
#*********** GLOBAL PARAMETERS **************
state = "init"
statemsg = False
#********************************************
def exitFunc():
    global cam
    global s
    global thread
    print("Escape hit, closing...")
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
        handOffPos = []
        global cam
        cam = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        offlineFlag = False
        start_time = time.time()
        template = cv2.imread('./images/vialTop.png', 0) # * Load template    
        cv2.namedWindow("Image")
        ret, img = cam.read()
        if not ret or img.shape != (720, 1280, 3):
            img = cv2.imread('./images/opencv_frame_6.png')
            offlineFlag = True
            cam.release()
        print("Press key to start, ESC to exit")
        k = cv2.waitKey(0)
        if k%256 == 27:
            exitFunc()
        if not offlineFlag:
            rl.robotInit()
        state = "sourceimg"
        
    if state == "sourceimg":
        if statemsg == False:
            print("Press key to process image, ESC to exit")
            statemsg = True
        if not offlineFlag:
            ret, img = cam.read()
        cv2.imshow("Image", img)
        k = cv2.waitKey(5)
        if k%256 == 27:
            exitFunc()
        elif k != -1:
            img_binary = prep.PrepImg(img)
            cv2.imshow("Binary image", img_binary)
            edges_hough = ls.HoughLinesSearch(img_binary)
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
                    cv2.imshow('Detection', final)
                    print("Press space to initiate pickup")
                    print("Press ESC to exit")
                    print("Press other to recapture image")
                    k = cv2.waitKey(0)
                    if k%256 == 27:
                        exitFunc()
                    if k%256 == 32:
                        state = "pickup"

    if state == "pickup":
        print('Initiating pickup at (x,y) = (%.1f,%.1f) cm' % (grabPoints[4], grabPoints[5]))
        
        state = "sourceimg"


#while True:
#    ret, img = cam.read()
#    k = cv2.waitKey(1)
#    if k%256 == 27:
#        # ESC pressed
#        print("Escape hit, closing...")
#        cam.release()
#        cv2.destroyAllWindows()
#    if not ret or img.shape != (720, 1280, 3):
#        img = cv2.imread('./images/opencv_frame_6.png')
#        offlineFlag = True
#        cam.release()
#    else:
#        cv2.imshow("Image", img)
#    
#    
#    
#    cv2.imshow("Image", img)
#    img_binary = prep.PrepImg(img)
#    cv2.imshow("Binary image", img_binary)
#    edges_hough = ls.HoughLinesSearch(img_binary)
#    if edges_hough is not None:
#        houghLocation = np.ndarray.flatten(edges_hough)
#        final = ls.templatematch(img_binary, template, houghLocation)
#        if final is not None:
#            grabPoints, grabAngle = ls.grabberPoint(houghLocation)
#            grabPoints = np.around(grabPoints)
#            grabPoints = grabPoints.astype(int)
#            cv2.circle(final, (grabPoints[0], grabPoints[1]), 3, color = (0,255,0), thickness=2)
#            cv2.circle(final, (grabPoints[2], grabPoints[3]), 3, color = (0,255,0), thickness=2)
#            cv2.circle(final, (grabPoints[4], grabPoints[5]), 3, color = (0,0,255), thickness=2)
#            print("--- %s seconds ---" % (time.time() - start_time))
#            cv2.imshow('Detection', final)
#            if offlineFlag == True:
#                break
#    else:
#        print("No lines found")
#
#cv2.waitKey(0)
#cam.release()
#cv2.destroyAllWindows()