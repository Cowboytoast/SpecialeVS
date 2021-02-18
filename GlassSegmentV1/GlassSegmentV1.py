import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
from scipy import ndimage
import pylab

# * Histogram stretching function
def HistStretch(img):
    tmp = img.copy()
    maxval = tmp.max()
    minval = tmp.min()
    for height in range(img.shape[0]):
        for width in range(img.shape[1]):
            tmp[height, width] = ((tmp[height, width] - minval) / (maxval - minval)) * 255
    return tmp

def SharpenImage(img, rad, amount, threshold):
    

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

def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result

# * Load image and resize
img = cv2.imread('IMG_0005_cropped.png')
img_screensized = ResizeToFit(img)

# * Chain should be:
# * Gray -> Hist. stretch -> median filtering (size 7, 23rd percentile) ...
# * -> sharpening (radius 3.058, amount 6.371, threshold 0.131) ...
# * -> Diff. of Gaussians (rad 1 3.912, rad 2: 6.463)

# * Grayscaling
img_gray = cv2.cvtColor(img_screensized, cv2.COLOR_BGR2GRAY)
img_stretched = HistStretch(img_gray)
img_percentile = ndimage.filters.percentile_filter(img_stretched, 23, (7,7))
img_sharpen = SharpenImage(img, 3, 6.4, 0.131)

'''
# * Filters
img_blurred = cv2.blur(img, (5,5))

# * Edge detection
edges = cv2.Canny(img_blurred, 5, 15)
edges_lownoise = RemoveNoise(edges, 5)


img_blurred = ResizeToFit(img_blurred)
'''
cv2.imshow('Low-pass', img_gray)
cv2.imshow('Original', img_stretched)
cv2.waitKey(0)
cv2.destroyAllWindows()