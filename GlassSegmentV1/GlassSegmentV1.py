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

def templatematch(img, template, houghLocation, h_steps = 20, w_steps = 20):
    # * line-pair = |slope1 = a rad | x1start | y1start | x1end | y1end | slope2 = b rad | x2start | y2start | x2end | y2end
    pointsx = np.array([houghLocation[1], houghLocation[3], houghLocation[6], houghLocation[8]])
    pointsy = np.array([houghLocation[2], houghLocation[4], houghLocation[7], houghLocation[9]])
    slopes = np.array([math.degrees(houghLocation[0]), math.degrees(houghLocation[5])])

    # Convert to positive slope angle
    for cnt in range(slopes.shape[0]):
        if np.sign(slopes[cnt]) == -1:
            slopes[cnt] += 360

    slope_offset = np.average(slopes)
    template_rot = imutils.rotate_bound(template, slope_offset)
    pointsy -= int(template_rot.shape[0]/2)
    pointsx -= int(template_rot.shape[1]/4)
    
    # Do & operation in increments, that is moving the template image a few pixels right/down
    # for each iteration and store most pixel hits
    maxval = 0
    UpDown = 1 # 1 for up, 0 for down
    
    for h in np.arange(int(np.amin(pointsy)) - int(h_steps/2), int(np.amin(pointsy)) + int(h_steps/2), 1):
        for w in np.arange(int(np.amin(pointsx)) - int(w_steps/2), int(np.amin(pointsx)) + int(w_steps/2), 1):
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            
            if matches > maxval:
                maxval = matches
                max_idx = np.array([h, w])
            #rotatingim = np.copy(img)
            #rotatingim[h : h + template_rot.shape[0], w : w + template_rot.shape[1]] = template_rot
            #cv2.imshow('Rotating progress', rotatingim)
            #cv2.waitKey(10)
    slope_offset += 180
    template_rot = imutils.rotate_bound(template, slope_offset)
    UpDown = 0 # 1 for up, 0 for down

    max_h = int(max_idx[0])
    max_w = int(max_idx[1])
    overlay = cv2.imread('VialOutline.png', 0)
    detection = imutils.rotate_bound(overlay, np.average(slopes))
    final = img
    final[max_h : max_h + detection.shape[0], max_w : max_w+detection.shape[1]] += detection
    
    return final
start_time = time.time()

# * Load image and convert to binary
img = cv2.imread('Frame4_Prep.png', 0)

# * Load template and convert to binary
template = cv2.imread('VialTop.png', 0)

# * Rotating template in increments
# * and match with image patch

houghLocation = np.array([math.radians(343), 186, 56, 225, 185, math.radians(343), 208, 51, 247, 179])
final = templatematch(img, template, houghLocation)
cv2.imwrite('TemplateResult.png', final)
print("--- %s seconds ---" % (time.time() - start_time))

# TODO Function with Mathias' program which sends:
# TODO line-pair = |slope1 = a rad | x1start | y1start | x1end | y1end | slope2 = b rad | x2start | y2start | x2end | y2end
cv2.imshow('bla', final)
cv2.waitKey(0)
cv2.destroyAllWindows()