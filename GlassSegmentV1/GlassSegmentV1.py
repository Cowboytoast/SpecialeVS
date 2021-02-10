import cv2
import numpy as np
import matplotlib.pyplot as plt
import time

# * Do simple morphology to remove noise from image
def RemoveNoise(img, dim = 3):
    kernel = np.ones((dim, dim))
    img_closed = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    img_opened = cv2.morphologyEx(img_closed, cv2.MORPH_OPEN, kernel)
    return img_opened

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

def ContourImage(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    drawing = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    for i in range(len(contours)):
        if contours[i].size > 50:
            cv2.drawContours(drawing, contours, i, (255, 255, 255), 2, cv2.LINE_8, hierarchy, 0)
    drawing = cv2.cvtColor(drawing, cv2.COLOR_BGR2GRAY)
    return drawing

def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result

# * Load image and resize
img = cv2.imread('IMG_0005_cropped.png')
img_screensized = ResizeToFit(img)

# * Template stuff
template = cv2.imread('VialOutline.png', 0)
scale_pct = 33.585 #33.585
template_width, template_height = template.shape[::-1]
template_width = int(template.shape[1] * scale_pct / 100)
template_height = int(template.shape[0] * scale_pct / 100)
dsize = (template_height, template_height)
template = cv2.resize(template, dsize)

# * Detect edges on image and remove noise
img_blurred = cv2.blur(img_screensized, (5,5))
cv2.imshow('filtered', img_blurred)
edges = cv2.Canny(img_blurred, 45, 45)
edges_lownoise = RemoveNoise(edges, 5)

# * Rotating template in 5 deg. increments
angle_inc = 5 # 5 deg. increment
for i in range(0, int(360/angle_inc)):
    template = rotate_image(template, angle_inc)
    res = cv2.matchTemplate(edges_lownoise, template, cv2.TM_CCORR_NORMED)
    threshold = 0.4
    loc = np.where(res >= threshold)
    for pt in zip(*loc[::-1]):
        cv2.rectangle(img_screensized, pt, (pt[0] + template_width, pt[1] + template_height), (0,0,255), 2)

cv2.imshow('Matches', img_screensized)
cv2.imshow('Contour image', edges_lownoise)
cv2.waitKey(0)
cv2.destroyAllWindows()