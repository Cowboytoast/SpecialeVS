import cv2
import numpy as np
from cv2 import aruco


#* Library for calibration between camera and box/world frame
#* Here we use ArUco markers to determind the corners of the box, corresponding to P0, Px and Py needed for the transformation between world frame and robot frame.

def markerCalib(img):
    dict = cv2.aruco.Dictionary_get(aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    markers,ids, _ = cv2.aruco.detectMarkers(img,dict,parameters=arucoParams)
    markers = np.array(markers, dtype = np.int32)
    corners_avg = np.empty([4,2], dtype = np.int32)
    if len(ids) < 4:
        return None

    rowNumber = np.zeros([4], dtype = np.int32)
    for i in range(0,len(ids)):
        if ids[i] == 1:
            rowNumber[0] = i
        elif ids[i] == 2:
            rowNumber[1] = i
        elif ids[i] == 3:
            rowNumber[2] = i
        elif ids[i] == 4:
            rowNumber[3] = i

    for cnt in range(0, 4):
        if cnt == 0:
            corners_avg[0, 0] = round(markers[rowNumber[0], 0, 0, 0] + markers[rowNumber[0], 0, 1, 0] + 
                            markers[rowNumber[0], 0, 2, 0] + markers[rowNumber[0], 0, 3, 0]) / 4 # x avg
            corners_avg[0, 1] = round(markers[rowNumber[0], 0, 0, 1] + markers[rowNumber[0], 0, 1, 1] + 
                            markers[rowNumber[0], 0, 2, 1] + markers[rowNumber[0], 0, 3, 1]) / 4 # y avg
        
        if cnt == 1:
            corners_avg[1, 0] = round(markers[rowNumber[1], 0, 0, 0] + markers[rowNumber[1], 0, 1, 0] + 
                                markers[rowNumber[1], 0, 2, 0] + markers[rowNumber[1], 0, 3, 0]) / 4
            corners_avg[1, 1] = round(markers[rowNumber[1], 0, 0, 1] + markers[rowNumber[1], 0, 1, 1] + 
                                markers[rowNumber[1], 0, 2, 1] + markers[rowNumber[1], 0, 3, 1]) / 4
        if cnt == 2:
            corners_avg[2, 0] = round(markers[rowNumber[2], 0, 0, 0] + markers[rowNumber[2], 0, 1, 0] + 
                                markers[rowNumber[2], 0, 2, 0] + markers[rowNumber[2], 0, 3, 0]) / 4
            corners_avg[2, 1] = round(markers[rowNumber[2], 0, 0, 1] + markers[rowNumber[2], 0, 1, 1] + 
                                markers[rowNumber[2], 0, 2, 1] + markers[rowNumber[2], 0, 3, 1]) / 4
        if cnt == 3:
            corners_avg[3, 0] = round(markers[rowNumber[3], 0, 0, 0] + markers[rowNumber[3], 0, 1, 0] + 
                                markers[rowNumber[3], 0, 2, 0] + markers[rowNumber[3], 0, 3, 0]) / 4
            corners_avg[3, 1] = round(markers[rowNumber[3], 0, 0, 1] + markers[rowNumber[3], 0, 1, 1] + 
                                markers[rowNumber[3], 0, 2, 1] + markers[rowNumber[3], 0, 3, 1]) / 4
    return corners_avg