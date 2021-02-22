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
    sortedLines = SortLines(linesP)
    LineGrouping = LinesGrouping(sortedLines)
    #If-statement drawing lines on the copy, if any lines are found.
    if linesP is not None:
        b = 255
        g = 0
        r = 0
        for i in range(0, len(LineGrouping)): #for all lines, insert linesP instead
            l = LineGrouping[i]
            l = l.astype(int)
            cv2.line(houghImage, (l[1], l[2]), (l[3], l[4]), (b,g,r), 3, cv2.LINE_AA)
            g=g+25

    return houghImage

def LinesGrouping(sortedLines):
    value = None
    lineGroup = []
    range_upper = 0
    j = 0
    k = 0
    glass0 = np.zeros([5,5])
    glass1 = np.zeros([5,5])
    glass2 = np.zeros([5,5])
    glass3 = np.zeros([5,5])
    glass4 = np.zeros([5,5])

    sortedLinesArray = np.array(sortedLines)
    np.set_printoptions(precision=6,suppress=True)
                
    for i in range(0,len(sortedLinesArray)):
        if value is None:
            value=sortedLinesArray[0,0]
            range_upper=value+0.45
            #first_flag = True
        elif range_upper < sortedLinesArray[i,0]:
            value=sortedLinesArray[i,0]
            range_upper=value+0.45
            j=j+1
            k=0
         
        if j == 0:
            glass0[k,0] = sortedLinesArray[i,0]
            glass0[k,1] = sortedLinesArray[i,1]
            glass0[k,2] = sortedLinesArray[i,2]
            glass0[k,3] = sortedLinesArray[i,3]
            glass0[k,4] = sortedLinesArray[i,4]
        elif j == 1:
            glass1[k,0] = sortedLinesArray[i,0]
            glass1[k,1] = sortedLinesArray[i,1]
            glass1[k,2] = sortedLinesArray[i,2]
            glass1[k,3] = sortedLinesArray[i,3]
            glass1[k,4] = sortedLinesArray[i,4]
        elif j == 2:
            glass2[k,0] = sortedLinesArray[i,0]
            glass2[k,1] = sortedLinesArray[i,1]
            glass2[k,2] = sortedLinesArray[i,2]
            glass2[k,3] = sortedLinesArray[i,3]
            glass2[k,4] = sortedLinesArray[i,4]
        elif j == 3:
            glass3[k,0] = sortedLinesArray[i,0]
            glass3[k,1] = sortedLinesArray[i,1]
            glass3[k,2] = sortedLinesArray[i,2]
            glass3[k,3] = sortedLinesArray[i,3]
            glass3[k,4] = sortedLinesArray[i,4]
        elif j == 4:
            glass4[k,0] = sortedLinesArray[i,0]
            glass4[k,1] = sortedLinesArray[i,1]
            glass4[k,2] = sortedLinesArray[i,2]
            glass4[k,3] = sortedLinesArray[i,3]
            glass4[k,4] = sortedLinesArray[i,4]
        k=k+1
    lineGroup=glass4
    print(glass2)
    return lineGroup

def SortLines(linesP):
    if linesP is not None:
        slope_sorted=[0 for i in range(len(linesP))]
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            x = [l[0], l[2]]
            y = [l[1], l[3]]
            slope = linregress(x,y)
            if slope.slope > 3.1415:
                slope_fixed = slope.slope#-3.1415
            elif slope.slope < -3.1415:
                slope_fixed = slope.slope#+3.1415
            else:
                slope_fixed = slope.slope
            #                 slope         x1   y1     x2   y2
            slope_sorted[i] = [slope_fixed,l[0],l[1],l[2],l[3]]
        slope_sorted.sort(key=lambda x:x[0])
        sortedLines = slope_sorted
    
    return sortedLines

#**********************Main loop***************************

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