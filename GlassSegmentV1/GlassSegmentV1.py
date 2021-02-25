import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import imutils
from scipy import ndimage

# * Resize image to fit screen while keeping aspect ratio
def ResizeToFit(oriimg):
    H, W = (980, 1820)
    height, width, depth = oriimg.shape
    scaleWidth = float(W)/float(width)
    scaleHeight = float(H)/float(height)

    if scaleHeight>scaleWidth:
        imgScale = scaleWidth
    else:
        imgScale = scaleHeight

    newX,newY = oriimg.shape[1]*imgScale, oriimg.shape[0]*imgScale
    newimg = cv2.resize(oriimg,(int(newX),int(newY)))
    return newimg

# ! Unused due to bounding-box non-scaliness. Use rotate_bound instead
def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_CUBIC)
    return result

# ! A (likely) ineffective way to convert image to binary
def cvt_to_bin(image):
    image_bin = np.zeros(image.shape,dtype=bool)
    for y in range(image.shape[0]):
        for x in range(image.shape[1]):
            if image[y,x] == 255:
                image_bin[y,x] = True
    return image_bin

# * Load image and convert to binary
img = cv2.imread('SimpleSegmentation.png', 0)
img_bin = cvt_to_bin(img)

# * Load template and convert to binary
template = cv2.imread('VialOutline.png', 0)
template_bin = cvt_to_bin(template)

# * Rotating template in 15 deg. increments
# * and match with image patch
angle_inc = 5 # 15 deg. increment
template_rot = template
matches = np.empty([1000, 2000])
# ? Perhaps scaling down the image significantly would 
# ? increase performance. But at what cost?

# Do & operation in increments, that is moving the template image a few pixels right/down
# for each iteration and store most pixel hits
# TODO: Extract sum of Trues for each width inc. and save in new array
    # TODO: Get array instead of single value when &'ing arrays
for angle in np.arange(0, 360, angle_inc):
    for h in np.arange(0, img.shape[0] - template_rot.shape[0], int(img.shape[0] / 25)):
        for w in np.arange(0, img.shape[1] - template_rot.shape[1], int(img.shape[1] / 25)):
            #i = 0       
            match_array = np.logical_and(img[h : h + template_rot.shape[0] : 1, w : w + template_rot.shape[1] : 1], template_rot)
            matches[h, w] = np.count_nonzero(match_array)
    template_rot = imutils.rotate_bound(template, angle)
    #cv2.imshow('bla',template_rot)
    #cv2.waitKey(50)
print(np.amax(match_array))
time.sleep(1)
#cv2.imshow('Binary', img)
#cv2.imshow('Contour image', edges_lownoise)
#cv2.waitKey(0)
#cv2.destroyAllWindows()