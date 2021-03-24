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

# ! An (likely) ineffective way to convert image to binary
def cvt_to_bin(image):
    image_bin = np.zeros(image.shape,dtype=bool)
    for y in range(image.shape[0]):
        for x in range(image.shape[1]):
            if image[y,x] == 255:
                image_bin[y,x] = True
    return image_bin

def templatematch(img, template, houghLocation, h_steps = 10, w_steps = 10):
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
    #matches = np.empty([int(10/angle_inc), img.shape[0] - template_rot.shape[1] + 10, img.shape[1] - template_rot.shape[1] + 10])
    
    maxval = 0

    # Do & operation in increments, that is moving the template image a few pixels right/down
    # for each iteration and store most pixel hits
    
    UpDown = 1 # 1 for up, 0 for down
    '''
    for h in np.arange(int(np.amin(pointsy)) - int(h_steps/2), int(np.amin(pointsy)) + int(h_steps/2), 1):
        for w in np.arange(int(np.amin(pointsx)) - int(w_steps/2), int(np.amin(pointsx)) + int(w_steps/2), 1):
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            
            if matches > maxval:
                maxval = matches
                max_idx = np.array([h, w])
            #template_rot = imutils.rotate_bound(template, angle)
            #rotatingim = np.copy(img)
            #rotatingim[h : h + templateTop_rot.shape[0], w : w + templateTop_rot.shape[1]] = templateTop_rot
            #cv2.imshow('Rotating progress', rotatingim)
            #cv2.waitKey(1)
                #matches[int(angle/angle_inc), h, w] = np.count_nonzero(match_array)
    slope_offset += 180
    template_rot = imutils.rotate_bound(template, slope_offset)
    UpDown = 0 # 1 for up, 0 for down


    #max_rot = int(max_idx[0])
    max_h = int(max_idx[0])
    max_w = int(max_idx[1])
    overlay = cv2.imread('VialOutline.png', 0)
    detection = imutils.rotate_bound(overlay, np.average(slopes))
    final = img
    final[max_h : max_h + detection.shape[0], max_w : max_w+detection.shape[1]] += detection
    #cv2.imwrite('IMG0005_Detection.png', final)
    '''
    img2 = img.copy()
    w, h = template_rot.shape[::-1]
    
    img = img2.copy()
    method = cv2.TM_CCORR
    res = cv2.matchTemplate(img, template_rot, method)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    top_left = max_loc
    bottom_right = (top_left[0] + w, top_left[1] + h)
    cv2.rectangle(img,top_left, bottom_right, 255, 2)
    plt.subplot(121),plt.imshow(res,cmap = 'gray')
    plt.title('Matching Result'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(img,cmap = 'gray')
    plt.title('Detected Point'), plt.xticks([]), plt.yticks([])
    plt.show()
    return img

start_time = time.time()

# * Load image and convert to binary
img = cv2.imread('PrepResult.png', 0)
#img = ResizeToFit(img)
#img_bin = cvt_to_bin(img)

# * Load template and convert to binary
template = cv2.imread('VialTop.png', 0)

# * Rotating template in increments
# * and match with image patch

houghLocation = np.array([math.radians(343), 186, 56, 225, 185, math.radians(343), 208, 51, 247, 179])
final = templatematch(img, template, houghLocation)
print("--- %s seconds ---" % (time.time() - start_time))
# TODO Prepare to function with line coordinates to narrow down position
# TODO Function with Mathias' program which sends:
# TODO line-pair = |slope1 = a rad | x1start | y1start | x1end | y1end | slope2 = b rad | x2start | y2start | x2end | y2end
# TODO Search in the point-wise vicinity, search within , e.g +- 10 deg, turn 180 deg, search again within +-10 deg, use best fit
cv2.imshow('bla1', img)
cv2.imshow('bla', final)
cv2.waitKey(0)
cv2.destroyAllWindows()