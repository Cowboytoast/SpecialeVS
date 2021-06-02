import cv2
from cv2 import aruco


#* Library for calibration between camera and box/world frame
#* Here we use ArUco markers to determind the corners of the box, corresponding to P0, Px and Py needed for the transformation between world frame and robot frame.

def markerCalib(img):
    
    dict = cv2.aruco.Dictionary_get(aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    corners,ids,refect = cv2.aruco.detectMarkers(img,dict,parameters=arucoParams)
    imgMarkers = cv2.aruco.drawDetectedMarkers(img.copy(),corners,ids)
    
    cv2.namedWindow("markers")
    cv2.imshow(imgMarkers,"markers")
    
    return corners,ids
