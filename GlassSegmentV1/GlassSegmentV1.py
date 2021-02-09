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

# * Do BLOB analysis on binary image
def BLOBanalysis(img):
    # ? Maybe findcontours is better
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 100
    params.filterByCircularity = False
    params.filterByColor = False
    params.filterByConvexity = False
    params.filterByInertia = False
    detector = cv2.SimpleBlobDetector_create(params)
    #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.bitwise_not(img)
    keypoints = detector.detect(img)
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), 
                                        cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    return im_with_keypoints

def ContourImage(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    drawing = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    for i in range(len(contours)):
        if contours[i].size > 50:
            cv2.drawContours(drawing, contours, i, (255, 255, 255), 2, cv2.LINE_8, hierarchy, 0)
        #cv2.imshow('window1', drawing)
        #cv2.waitKey(100)
    #print(contours[4].size)
    drawing = cv2.cvtColor(drawing, cv2.COLOR_BGR2GRAY)
    return drawing

print("test")
# * Load image and resize
img = cv2.imread('IMG_0005_cropped.png')
img_screensized = ResizeToFit(img)

# * Template stuff
template = cv2.imread('VialOutline.png', 0)
scale_pct = 33.585
template_width, template_height = template.shape[::-1]
template_width = int(template.shape[1] * scale_pct / 100)
template_height = int(template.shape[0] * scale_pct / 100)
dsize = (template_width, template_height)
template = cv2.resize(template, dsize)
template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

# * Show unaltered image
#cv2.imshow('image_window',img_screensized)

# * Detect edges on image and remove noise
edges = cv2.Canny(img_screensized, 40, 40)
edges_lownoise = RemoveNoise(edges, 5)
#BLOBs = BLOBanalysis(edges_lownoise)
contours = ContourImage(edges_lownoise)
res = cv2.matchTemplate(contours, template, cv2.TM_CCORR_NORMED)
threshold = 0.3
loc = np.where(res >= threshold)
added_image = img_screensized
for pt in zip(*loc[::-1]):
        cv2.rectangle(img_screensized, pt, (pt[0] + template_width, pt[1] + template_height), (0,0,255), 2)

cv2.imshow('Matching result', res)
cv2.imshow('edge_window',img_screensized)
cv2.waitKey(0)
cv2.destroyAllWindows()