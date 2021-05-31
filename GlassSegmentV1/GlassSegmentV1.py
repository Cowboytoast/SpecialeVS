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
angleTolerance = 0.3
extractCounter = 0
handOffPos = []
cam = cv2.VideoCapture(1)

#********************************************


#**********************Main loop********************************
rl.robotInit()
start_time = time.time()
template = cv2.imread('./images/vialTop.png', 0) # * Load template
#***************************************************************
cv2.namedWindow("Image")
print("Press key to start, ESC to exit")

while True:
    rl.robotRun()
    k = cv2.waitKey(1)
    if k%256 == 27:
        break
    
while True:
    ret, img = cam.read()
    k = cv2.waitKey(1)
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        cam.release()
        cv2.destroyAllWindows()
    if not ret or img.shape != (720, 1280, 3):
        img = cv2.imread('./images/opencv_frame_6.png')
        offlineFlag = True
    else:
        cv2.imshow("Image", img)
    
    cv2.imshow("Image", img)
    img_binary = prep.PrepImg(img)
    cv2.imshow("Binary image", img_binary)
    edges_hough = ls.HoughLinesSearch(img_binary)
    if edges_hough is not None:
        houghLocation = np.ndarray.flatten(edges_hough)
        final = ls.templatematch(img_binary, template, houghLocation)
        if final is not None:
            grabPoints, grabAngle = ls.grabberPoint(houghLocation)
            grabPoints = np.around(grabPoints)
            grabPoints = grabPoints.astype(int)
            cv2.circle(final, (grabPoints[0], grabPoints[1]), 3, color = (0,255,0), thickness=2)
            cv2.circle(final, (grabPoints[2], grabPoints[3]), 3, color = (0,255,0), thickness=2)
            cv2.circle(final, (grabPoints[4], grabPoints[5]), 3, color = (0,0,255), thickness=2)
            print("--- %s seconds ---" % (time.time() - start_time))
            cv2.imshow('Detection', final)
            if offlineFlag == True:
                break
    else:
        print("No lines found")
        
cv2.waitKey(0)
cam.release()
cv2.destroyAllWindows()