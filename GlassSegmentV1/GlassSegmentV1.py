import cv2
import time
import sys
sys.path.append("./images")
from cv2 import aruco

import LineSearchLib as ls
import PreprocessingLib as prep
#import RobotLib as rl
import numpy as np
from scipy import ndimage
# TODO: Fix line merge. Der sker en fejl ved billede opencv_frame_1
# TODO: Lav movement filer om således at vi har et 'init' call og kan kalde de andre filer med argumenter for position.
#*********** GLOBAL PARAMETERS **************
angleTolerance = 0.3
extractCounter = 0
handOffPos = []
cam = cv2.VideoCapture(0)

#********************************************


#**********************Main loop********************************
start_time = time.time()
template = cv2.imread('./images/vialTop.png', 0) # * Load template
#***************************************************************


cv2.namedWindow("Camera feedback")

img_counter = 0

while True:
    ret, img = cam.read()
    print("Press key to start, ESC to exit")
    k = cv2.waitKey(0)
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
        cam.release()
        cv2.destroyAllWindows()
        exit()
    if not ret or frame.shape == (576, 1024, 3):
        print("Failed to grab frame")
        print("Offline image used")
        img = cv2.imread('./images/opencv_frame_6.png')
        break
    cv2.imshow("Image", img)
    img_name = "opencv_frame_{}.png".format(img_counter)
    cv2.imwrite(img_name, img)
    print("opencv_frame_{} written!".format(img_name))
    img_counter += 1
    img_binary = prep.PrepImg(img)
    edges_hough = ls.HoughLinesSearch(img_binary)
    houghLocation = np.ndarray.flatten(edges_hough)
    final = ls.templatematch(img_binary, template, houghLocation)
    # TODO Rename grabPoints to something nicer
    grabPoints, grabAngle = ls.grabberPoint(houghLocation)
    grabPoints = np.around(grabPoints)
    grabPoints = grabPoints.astype(int)
    cv2.circle(final, (grabPoints[0], grabPoints[1]), 3, color = (0,255,0), thickness=2)
    cv2.circle(final, (grabPoints[2], grabPoints[3]), 3, color = (0,255,0), thickness=2)
    cv2.circle(final, (grabPoints[4], grabPoints[5]), 3, color = (0,0,255), thickness=2)
    print("--- %s seconds ---" % (time.time() - start_time))
    cv2.imshow('Detection', final)

cam.release()
cv2.destroyAllWindows()
cv2.waitKey(0)
cv2.destroyAllWindows()