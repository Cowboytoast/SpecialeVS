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
    height, width = oriimg.shape
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

def templatematch(img, template, angle_inc = 5, h_steps = 25, w_steps = 40):
    template_rot = template
    matches = np.empty([int(360/angle_inc), img.shape[0] - template_rot.shape[1] + 1, img.shape[1] - template_rot.shape[1] + 1])
    #rotations = np.empty([])
    # ? Perhaps scaling down the image significantly would 
    # ? increase performance. But at what cost?

    # Do & operation in increments, that is moving the template image a few pixels right/down
    # for each iteration and store most pixel hits
    for angle in np.arange(0, 360, angle_inc):
        for h in np.arange(0, img.shape[0] - template_rot.shape[0], int(img.shape[0] / h_steps)):
            for w in np.arange(0, img.shape[1] - template_rot.shape[1], int(img.shape[1] / w_steps)):
                match_array = np.logical_and(img[h : h + template_rot.shape[0] : 1, w : w + template_rot.shape[1] : 1], template_rot)
                matches[int(angle/angle_inc), h, w] = np.count_nonzero(match_array)
        template_rot = imutils.rotate_bound(template, angle)

    max_idx = np.where(matches == np.amax(matches))
    max_h = np.amax(max_idx[1])
    max_w = np.amax(max_idx[2])
    detection = imutils.rotate_bound(template, np.amax(max_idx[0]) * angle_inc)
    final = img
    final[max_h:max_h + detection.shape[0]:1, max_w:max_w+detection.shape[1]:1] += detection
    return final

# * Load image and convert to binary
img = cv2.imread('IMG_0005_opt3.png', 0)
img = ResizeToFit(img)
#img_bin = cvt_to_bin(img)

# * Load template and convert to binary
template = cv2.imread('VialOutlineHollow.png', 0)

# * Rotating template in increments
# * and match with image patch
final = templatematch(img, template, angle_inc = 5, w_steps = 40, h_steps = 25)

# TODO Prepare to function with line coordinates to narrow down position
# TODO Function with Mathias' program which sends:
# TODO line-pair = |slope1 = a rad | x1start | y1start | x1end | y1end | slope2 = b rad | x2start | y2start | x2end | y2end
# TODO Search in the point-wise vicinity, search within , e.g +- 10 deg, turn 180 deg, search again within +-10 deg, use best fit
cv2.imshow('bla1', img)
cv2.imshow('bla', final)

#cv2.imshow('Binary', img)
#cv2.imshow('Contour image', edges_lownoise)
cv2.waitKey(0)
cv2.destroyAllWindows()