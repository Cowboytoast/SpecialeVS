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
    corners = np.empty([4,1,2], dtype = np.int32)
    for cnt in range(0,4):
        ID = ids[cnt] - 1
        if ID == 0:
            corners[0] = markers[0, 0, 0, :]
        if ID == 1:
            corners[1] = markers[1, 0, 1, :]
        if ID == 2:
            corners[2] = markers[2, 0, 2, :]
        if ID == 3:
            corners[3] = markers[3, 0, 3, :]
    
    cv2.namedWindow("markers")
    cv2.imshow("markers",imgMarkers)
    
    return corners,ids

def markerCrop(img,corners):
    #https://jdhao.github.io/2019/02/23/crop_rotated_rectangle_opencv/
    #? We might need to acsociate the ids with a specifik corner
    #? to know what placement in the 'corners' array is belonging to what corner.

    corners1 = np.array([
            [[64, 49]],
            [[122, 11]],
            [[391, 326]],
            [[308, 373]]
        ])
    print("shape of cnt: {}".format(corners.shape))
    rect = cv2.minAreaRect(corners)
    print("rect: {}".format(rect))

    # the order of the box points: bottom left, top left, top right,
    # bottom right
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    print("bounding box: {}".format(box))
    #cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

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

    # cv2.imwrite("crop_img.jpg", warped)
    cv2.imshow("warppp", warped)
    cv2.waitKey(0)
        
    return warped