import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import linregress

def HoughLinesSearch(img, houghLength=40, houghDist=10):
    #img has to be the edge detected image.
    #Copy of edge detected image into BGR image for drawing lines.
    houghImage = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    #Find HoughLines on the image. Default houghLengt = 40, houghDist=10
    linesP = cv2.HoughLinesP(img, 1, np.pi / 180, 50, None, houghLength, houghDist)
    #If-statement drawing lines on the copy, if any lines are found.
    if linesP is not None:
        slope_sorted=[0 for i in range(len(linesP))]
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(houghImage, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
            x = [l[0], l[2]]
            y = [l[1], l[3]]
            slope = linregress(x,y)
            slope_sorted[i] = slope.slope
        slope_sorted.sort()
        print("slopes: ")
        print(slope_sorted)
        print("number of lines found: ")
        print(len(linesP))
    return houghImage

def lineDistance(linesP):
    #This fundtion finds the distance between two lines, calculated from the HoughLinesP() function.
        
    return dist

img = cv2.imread('IR_test_cropped.png')

imwidth = img.shape[1]
imheight = img.shape[0]

#des_width = int(imwidth * 0.25)
#des_height = int(imheight * 0.25)
des_dim = (imwidth, imheight)
img_screensized = cv2.resize(img, des_dim, interpolation=cv2.INTER_LANCZOS4)

cv2.imshow('image_window',img_screensized)
edges = cv2.Canny(img_screensized, 45, 45)
kernel = np.ones((5,5))

edges_hough = HoughLinesSearch(edges)

#Add calculations of the distance between lines, and the angle bestween lines. Use this to decide if to lines belong to the same vial.

cv2.imshow('edge_window',edges)
cv2.imshow('edge_hough',edges_hough)
cv2.waitKey(0)
cv2.destroyAllWindows()