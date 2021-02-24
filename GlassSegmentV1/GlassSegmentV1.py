import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import pylab
from scipy import ndimage
from skimage.filters import difference_of_gaussians
from skimage.util import img_as_ubyte

# * Histogram stretching function
def HistStretch(img):
    tmp = img.copy()
    maxval = tmp.max()
    minval = tmp.min()
    for height in range(img.shape[0]):
        for width in range(img.shape[1]):
            tmp[height, width] = ((tmp[height, width] - minval) / (maxval - minval)) * 255
    return tmp

def DiffOfGauss(image, rad1, rad2):
    size1 = rad1 * 2 + 1
    size2 = rad2 * 2 + 1
    img1 = cv2.GaussianBlur(image, (size1, size1), 0)
    img2 = cv2.GaussianBlur(image, (size2, size2), 0)
    DoG = (img1 - img2)
    return DoG

# https://stackoverflow.com/questions/4993082/how-can-i-sharpen-an-image-in-opencv
def unsharp_mask(image, kernel_size=(5, 5), sigma=1.0, amount=1.0, threshold=0):
    """Return a sharpened version of the image, using an unsharp mask."""
    blurred = cv2.GaussianBlur(image, kernel_size, sigma)
    sharpened = float(amount + 1) * image - float(amount) * blurred
    sharpened = np.maximum(sharpened, np.zeros(sharpened.shape))
    sharpened = np.minimum(sharpened, 255 * np.ones(sharpened.shape))
    sharpened = sharpened.round().astype(np.uint8)
    if threshold > 0:
        low_contrast_mask = np.absolute(image - blurred) < threshold
        np.copyto(sharpened, image, where=low_contrast_mask)
    return sharpened

def image_threshold(image, lower, upper = 255):
    tmp = image.copy()
    for height in range(tmp.shape[0]):
        for width in range(tmp.shape[1]):
            if tmp[height, width] < lower:
                tmp[height, width] = 0
            elif tmp[height, width] > upper:
                tmp[height, width] = 0
            else:
                tmp[height, width] = 255  
    return tmp

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
#img_sharpen = SharpenImage(img, 3, 6.4, 0.131)
img_sharpen = unsharp_mask(img_percentile, kernel_size = (3,3), amount = 6.4, threshold = 0.131)
img_edges = difference_of_gaussians(img_sharpen, 2, 8)
img_edges = img_as_ubyte(img_edges)
img_binary = image_threshold(img_edges, 4)
cv2.imshow('Original', img_screensized)
cv2.imshow('Gray', img_gray)
cv2.imshow('Stretched Hist.', img_stretched)
cv2.imshow('Percentile', img_percentile)
cv2.imshow('Sharp', img_sharpen)
cv2.imshow('DoG', img_edges)
cv2.imshow('Binary', img_binary)
'''
# * Filters
img_blurred = cv2.blur(img, (5,5))

# * Edge detection
edges = cv2.Canny(img_blurred, 5, 15)
edges_lownoise = RemoveNoise(edges, 5)


img_blurred = ResizeToFit(img_blurred)
'''
cv2.waitKey(0)
cv2.destroyAllWindows()