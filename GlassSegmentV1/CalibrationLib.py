import cv2
import numpy as np
from cv2 import aruco


#* Library for calibration between camera and box/world frame
#* Here we use ArUco markers to determind the corners of the box, corresponding to P0, Px and Py needed for the transformation between world frame and robot frame.

def markerCalib(img):
    dict = cv2.aruco.Dictionary_get(aruco.DICT_4X4_50)
    arucoParams = cv2.aruco.DetectorParameters_create()
    markers,ids,reject = cv2.aruco.detectMarkers(img,dict,parameters=arucoParams)
    imgMarkers = cv2.aruco.drawDetectedMarkers(img.copy(),markers,ids)
    markers = np.array(markers, dtype = np.int32)
    corners_4x4 = np.empty([4,1,2], dtype = np.int32)
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
            #corners_4x4[0] = markers[0, 0, 2, :]
        if cnt == 0:
            corners_avg[0, 0] = round(markers[rowNumber[0], 0, 0, 0] + markers[rowNumber[0], 0, 1, 0] + 
                            markers[rowNumber[0], 0, 2, 0] + markers[rowNumber[0], 0, 3, 0]) / 4 # x avg
            corners_avg[0, 1] = round(markers[rowNumber[0], 0, 0, 1] + markers[rowNumber[0], 0, 1, 1] + 
                            markers[rowNumber[0], 0, 2, 1] + markers[rowNumber[0], 0, 3, 1]) / 4 # y avg
        
        if cnt == 1:
            #corners_4x4[1] = markers[1, 0, 3, :]
            corners_avg[1, 0] = round(markers[rowNumber[1], 0, 0, 0] + markers[rowNumber[1], 0, 1, 0] + 
                                markers[rowNumber[1], 0, 2, 0] + markers[rowNumber[1], 0, 3, 0]) / 4
            corners_avg[1, 1] = round(markers[rowNumber[1], 0, 0, 1] + markers[rowNumber[1], 0, 1, 1] + 
                                markers[rowNumber[1], 0, 2, 1] + markers[rowNumber[1], 0, 3, 1]) / 4
        if cnt == 2:
            #corners_4x4[2] = markers[2, 0, 0, :]
            corners_avg[2, 0] = round(markers[rowNumber[2], 0, 0, 0] + markers[rowNumber[2], 0, 1, 0] + 
                                markers[rowNumber[2], 0, 2, 0] + markers[rowNumber[2], 0, 3, 0]) / 4
            corners_avg[2, 1] = round(markers[rowNumber[2], 0, 0, 1] + markers[rowNumber[2], 0, 1, 1] + 
                                markers[rowNumber[2], 0, 2, 1] + markers[rowNumber[2], 0, 3, 1]) / 4
        if cnt == 3:
            #corners_4x4[3] = markers[3, 0, 1, :]
            corners_avg[3, 0] = round(markers[rowNumber[3], 0, 0, 0] + markers[rowNumber[3], 0, 1, 0] + 
                                markers[rowNumber[3], 0, 2, 0] + markers[rowNumber[3], 0, 3, 0]) / 4
            corners_avg[3, 1] = round(markers[rowNumber[3], 0, 0, 1] + markers[rowNumber[3], 0, 1, 1] + 
                                markers[rowNumber[3], 0, 2, 1] + markers[rowNumber[3], 0, 3, 1]) / 4

    #corners_avg = corners_avg[sort, :]

    return corners_avg

def markerCrop(img,corners):
    #https://jdhao.github.io/2019/02/23/crop_rotated_rectangle_opencv/
    #? We might need to acsociate the ids with a specifik corner
    #? to know what placement in the 'corners' array is belonging to what corner.
    rect = cv2.minAreaRect(corners)

    # the order of the box points: bottom left, top left, top right,
    # bottom right
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    # get width and height of the detected rectangle
    width = int(rect[1][0])
    height = int(rect[1][1])

    src_pts = box.astype("float32")
    # coordinate of the points in box points after the rectangle has been
    # straightened
    dst_pts = np.array([[0, height-1],
                        [0, 0],
                        [width-1, 0],
                        [width-1, height-1]], dtype="float32")

    # the perspective transformation matrix
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    # directly warp the rotated rectangle to get the straightened rectangle
    warped = cv2.warpPerspective(img, M, (width, height))

    return warped