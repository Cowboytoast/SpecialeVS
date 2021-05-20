import cv2
import time
from cv2 import aruco
import LineSearchLib as ls
import PreprocessingLib as prep
import RobotLib as rl
from skimage.util import img_as_ubyte
import numpy as np



from scipy import ndimage
# TODO: Fix line merge. Der sker en fejl ved billede opencv_frame_1
# TODO: Lav movement filer om således at vi har et 'init' call og kan kalde de andre filer med argumenter for position.
#*********** GLOBAL PARAMETERS **************
angleTolerance = 0.3
extractCounter = 0
handOffPos = []

#********************************************


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
# * Threshold @ 32
start_time = time.time()

img = cv2.imread('opencv_frame_6.png')
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
# ! No, Otsu's method is way too sensitive to slight changes
# ! Also, in a fixed setup like this, we do not need the automatic detection
img_binary = cv2.threshold(img_edges, 32, maxval = 255, type = cv2.THRESH_BINARY)
img_binary = img_binary[1]
#cv2.imshow('Preprocessing', img_binary)
edges_hough = ls.HoughLinesSearch(img_binary)
#***************************************************************



houghLocation = np.ndarray.flatten(edges_hough)
final = ls.templatematch(img_binary, template, houghLocation)
cv2.imwrite('Bla.png',final)
# TODO Rename grabPoints to something nicer
grabPoints, grabAngle = ls.grabberPoint(houghLocation)
grabPoints = np.around(grabPoints)
grabPoints = grabPoints.astype(int)
cv2.circle(final, (grabPoints[0], grabPoints[1]), 3, color = (0,255,0), thickness=2)
cv2.circle(final, (grabPoints[2], grabPoints[3]), 3, color = (0,0,255), thickness=2)
cv2.circle(final, (grabPoints[4], grabPoints[5]), 3, color = (255,0,0), thickness=2)
print("--- %s seconds ---" % (time.time() - start_time))

cv2.imwrite('Test.png', final)

cv2.imshow('Detection', final)
cv2.waitKey(0)
cv2.destroyAllWindows()