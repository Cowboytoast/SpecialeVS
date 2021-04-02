import cv2
import time
import LineSearchLib as ls
import PreprocessingLib as prep
from skimage.util import img_as_ubyte
import numpy as np
import math
import imutils
from scipy import ndimage

#*********** GLOBAL PARAMETERS **************
angleTolerance = 0.3

#********************************************

def templatematch(img, template, houghLocation, h_steps = 10, w_steps = 10):
    # * line-pair = |slope1 = a rad | x1start | y1start | x1end | y1end | slope2 = b rad | x2start | y2start | x2end | y2end
    if houghLocation.size== 0:
        print("One or no lines found!")
        exit()
        
    pointsx = np.array([houghLocation[1], houghLocation[3], houghLocation[7], houghLocation[9]])
    pointsy = np.array([houghLocation[2], houghLocation[4], houghLocation[8], houghLocation[10]])
    slopes = np.array([houghLocation[0], houghLocation[6]])
    
    slope_offset = math.degrees(math.atan(np.average(slopes)))
    # Convert to positive slope angle
    #PosAng = 180 - abs(slope_offset)
    slope_offset = 90 - abs(slope_offset)
    if slope_offset < 0:
        slope_offset += 45
    
    
    
    #slope_offset = 27
    template_rot = imutils.rotate_bound(template, slope_offset)
    startPoint = np.argmin(pointsx + pointsy)
    
    pointsy -= int(template_rot.shape[0]/2)
    pointsx -= int(template_rot.shape[1]/4)
    
    # Do & operation in increments, that is moving the template image a few pixels right/down
    # for each iteration and store most pixel hits
    maxval = 0
    UpDown = 1 # 1 for up, 0 for down
    max_idx = np.zeros((2,1))
    #for h in np.arange(int()
    for h in np.arange(int(pointsy[startPoint]) - int(h_steps/2), int(pointsy[startPoint]) + int(h_steps/2), 1):
        for w in np.arange(int(pointsx[startPoint]) - int(w_steps/2), int(pointsx[startPoint]) + int(w_steps/2), 1):
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            
            if matches >= maxval:
                maxval = matches
                max_idx = np.array([h, w])
            #rotatingim = np.copy(img)
            #rotatingim[h : h + template_rot.shape[0], w : w + template_rot.shape[1]] = template_rot
            #cv2.imshow('Rotating progress', rotatingim)
            #cv2.waitKey(10)

    startPoint = np.argmax(pointsx + pointsy)
    template_rot = imutils.rotate_bound(template_rot, 180) # Rotate template by 180 deg
    
    #pointsy -= int(template_rot.shape[0]/4)
    pointsx -= int(template_rot.shape[1]/2)
    for h in np.arange(int(pointsy[startPoint]) + int(h_steps/2), int(pointsy[startPoint]) - int(h_steps/2), -1):
        for w in np.arange(int(pointsx[startPoint]) + int(w_steps/2), int(pointsx[startPoint]) - int(w_steps/2), -1):
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            
            if matches >= maxval:
                UpDown = 0 # 1 for up, 0 for down
                maxval = matches
                max_idx = np.array([h, w])
            rotatingim = np.copy(img)
            rotatingim[h : h + template_rot.shape[0], w : w + template_rot.shape[1]] = template_rot
            cv2.imshow('Rotating progress', rotatingim)
            cv2.waitKey(20)

    max_h = int(max_idx[0])
    max_w = int(max_idx[1])


    # * OVERLAY STUFF***************************************
    overlay = cv2.imread('VialOutline.png', 0)
    final = np.copy(img)
    templateStartH, templateStartW = shiftIdx(template_rot)
    
    # * To overlay entire vial use code below
    # * To overlay template use code below
    if UpDown == 1:
        template_rot = imutils.rotate_bound(template_rot, 180)
        detection = imutils.rotate_bound(overlay, slope_offset)
        final[max_h : max_h + template_rot.shape[0],
        max_w : max_w + template_rot.shape[1]] += template_rot
        overlayStartH, overlayStartW = shiftIdx(detection)
        adjustH = int(-templateStartH - overlayStartH)
        adjustW = int(templateStartW - overlayStartW)
        final[max_h + adjustH : max_h + detection.shape[0] + adjustH, 
            max_w + adjustW : max_w + detection.shape[1] + adjustW] += detection
    else:
        detection = imutils.rotate_bound(overlay, slope_offset + 180)
        final[max_h : max_h + template_rot.shape[0],
        max_w : max_w + template_rot.shape[1]] += template_rot
        overlayStartH, overlayStartW = shiftIdx(detection)
        adjustH = int(templateStartH - overlayStartH)
        adjustW = int(templateStartW - overlayStartW)
        #final[max_h + template_rot.shape[0] : max_h  - detection.shape[0], 
        #    max_w + template_rot.shape[1] : max_w + template_rot.shape[1] - detection.shape[1]] += detection
    
    
    
    return final

def shiftIdx(array):
    # * Finds first non-zero value in a 2D array or 2D array of arrays
    for H in range(array.shape[0]):
        for W in range(array.shape[1]):
            if array[H, W] > 0:
                return H, W

#**********************Main loop********************************

'''
#*****************FOR MATHIAS' IR IMAGE USE:********************
img = cv2.imread('IR_test_cropped.png')
imwidth = img.shape[1]
imheight = img.shape[0]
des_dim = (imwidth, imheight)
img_screensized = cv2.resize(img, des_dim, interpolation=cv2.INTER_LANCZOS4)

edges = cv2.Canny(img_screensized, 45, 45)
edges_hough = HoughLinesSearch(edges)
print("--- %s seconds ---" % (time.time()-start_time))
cv2.imshow('edge_hough',edges_hough)
cv2.waitKey(0)
cv2.destroyAllWindows()
#***************************************************************
'''
#**************FOR FREDERIK'S VISUAL IMAGE USE:*****************
# * Chain should be:
# * Crop -> Resize to (H,W = 403, 550) -> Cvt to gray -> ...
# * Hist. stretch -> Blur (rad. = (3,3), SigmaX = 7) -> ...
# * Unsharp mask (Size (3,3), amount 1, thresh. .131) -> ...
# * Laplacian edge (delta = 5) -> Cvt. to UByte -> ...
# * Threshold @ 30
start_time = time.time()

img = cv2.imread('opencv_frame_1R.png')
template = cv2.imread('vialTop.png', 0) # * Load template

img_cropped = img[60:60+505, 325:325+740]
img_screensized = prep.ResizeToFit(img_cropped, H= 403, W = 550)
img_gray = cv2.cvtColor(img_screensized, cv2.COLOR_BGR2GRAY)
img_stretched = cv2.equalizeHist(img_gray)
img_blur = cv2.GaussianBlur(img_stretched, (3,3), 7)
img_sharpen = prep.unsharp_mask(img_blur, kernel_size = (3,3), amount = 1, threshold = 0.131)
img_edges = cv2.Laplacian(img_sharpen, ddepth = cv2.CV_16S, delta = 5)
img_edges = img_as_ubyte(img_edges)
# ! Consider using Otsu's binarization THRESH_OTSU instead
img_binary = cv2.threshold(img_edges, 30, maxval = 255, type = cv2.THRESH_OTSU)
img_binary = img_binary[1]
#upscaled = prep.ResizeToFit(img_binary, H = 720, W = 1080)
#cv2.imshow('Preprocessing', img_binary)
#cv2.waitKey(0)
edges_hough = ls.HoughLinesSearch(img_binary)
#print("--- %s seconds ---" % (time.time()-start_time))

#cv2.imshow('edge_hough',edges_hough)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
#***************************************************************



houghLocation = np.ndarray.flatten(edges_hough)
#houghLocation[0] = math.radians(36)
#houghLocation[5] = math.radians(36)
#houghLocation = np.array([math.radians(343), 186, 56, 225, 185, math.radians(343), 208, 51, 247, 179])
final = templatematch(img_binary, template, houghLocation)
print("--- %s seconds ---" % (time.time() - start_time))

# TODO Function with Mathias' program which sends:
# TODO line-pair = |slope1 = a rad | x1start | y1start | x1end | y1end | slope2 = b rad | x2start | y2start | x2end | y2end
cv2.imshow('bla', final)
cv2.waitKey(0)
cv2.destroyAllWindows()