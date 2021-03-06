import cv2
import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np
import time
from scipy.stats import linregress
from skimage.transform import hough_line

# TODO: Lær at forstå hvordan matplot virker.
#? Matplot giver stadig ikke mening når det kommer til Hough space.
start_time = time.time()
def HoughLinesSearchSkimage(img):
    
    minDist = 20
    maxDist = 50
    h,theta,d = hough_line(img)
    
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    ax = axes.ravel()

    ax[0].imshow(img, cmap=cm.gray)
    ax[0].set_title('Input image')
    ax[0].set_axis_off()

    angle_step = 0.5 * np.diff(theta).mean()
    d_step = 0.5 * np.diff(d).mean()
    bounds = [np.rad2deg(theta[0] - angle_step),
            np.rad2deg(theta[-1] + angle_step),
            d[-1] + d_step, d[0] - d_step]
    ax[1].imshow(np.log(1 + h), extent=bounds, cmap=cm.gray, aspect=1 / 1.5)
    ax[1].set_title('Hough transform')
    ax[1].set_xlabel('Angles (degrees)')
    ax[1].set_ylabel('Distance (pixels)')
    ax[1].axis('image')

    plt.tight_layout()
    plt.show()
    
    houghImage = img
    
    return houghImage

def HoughLinesSearch(img, houghLength=40, houghDist=10):
    #img has to be the edge detected image.
    #Copy of edge detected image into BGR image for drawing lines.
    houghImage = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    #Find HoughLines on the image. Default houghLengt = 40, houghDist=10
    linesP = cv2.HoughLinesP(img, 1, np.pi / 180, 50, None, houghLength, houghDist)
    #If-statement drawing lines on the copy, if any lines are found.
    if linesP is not None:
        sortedLines = SortLines(linesP)
        LineGrouping = LinesGrouping(sortedLines)
        glassSides = LineMerge(LineGrouping) 
        b = 255
        g = 0
        r = 0
        for i in range(0, len(glassSides)): #for all lines: "linesP", for one glass all lines: "LineGrouping"
            l = glassSides[i]
            l = l.astype(int)
            cv2.line(houghImage, (l[1], l[2]), (l[3], l[4]), (b,g,r), 3, cv2.LINE_AA)
            g+=-255
            r+=255
    else:
        print("No lines found")
    return houghImage

def LineMerge(glassLines):
    # * function that merge the lines of a side to only one line
    lineMerged = np.zeros([2,6])
    k = 0
    if len(glassLines) == 2: # check if there exist only 2 lines
        a0 = abs(glassLines[0,1]-glassLines[0,3])
        b0 = abs(glassLines[0,2]-glassLines[0,4])
        a1 = abs(glassLines[1,1]-glassLines[1,3])
        b1 = abs(glassLines[1,2]-glassLines[1,4])
        c0 = np.hypot(a0,b0)
        c1 = np.hypot(a1,b1)
        lineMerged[0,0] = glassLines[0,0]
        lineMerged[0,1] = glassLines[0,1]
        lineMerged[0,2] = glassLines[0,2]
        lineMerged[0,3] = glassLines[0,3]
        lineMerged[0,4] = glassLines[0,4]
        lineMerged[0,5] = c0
        lineMerged[1,0] = glassLines[1,0]
        lineMerged[1,1] = glassLines[1,1]
        lineMerged[1,2] = glassLines[1,2]
        lineMerged[1,3] = glassLines[1,3]
        lineMerged[1,4] = glassLines[1,4]
        lineMerged[1,5] = c1
    elif len(glassLines) == 3: # check if there exist only 3 lines
        angleRangeLower = glassLines[0,0]-0.2
        angleRangeUpper = glassLines[0,0]+0.2
        
        x0Start = glassLines[0,1]
        y0Start = glassLines[0,2]
        x0End = glassLines[0,3]
        y0End = glassLines[0,4]
            
        x1Start = glassLines[1,1]
        y1Start = glassLines[1,2]
        x1End = glassLines[1,3]
        y1End = glassLines[1,4]
            
        x2Start = glassLines[2,1]
        y2Start = glassLines[2,2]
        x2End = glassLines[2,3]
        y2End = glassLines[2,4]
            
        slope01 = linregress([x0Start,x1End],[y0Start,y1End])
        slope02 = linregress([x0Start,x2End],[y0Start,y2End])
        slope12 = linregress([x1Start,x2End],[y1Start,y2End])
        
        slope01 = slope01.slope
        slope02 = slope02.slope
        slope12 = slope12.slope
            
        if (slope01 > angleRangeLower) and (slope01 < angleRangeUpper):
            a1 = abs(glassLines[0,1]-glassLines[1,3])
            b1 = abs(glassLines[0,2]-glassLines[1,4])
            c1 = np.hypot(a1,b1)
            a2 = abs(glassLines[1,1]-glassLines[0,3])
            b2 = abs(glassLines[1,2]-glassLines[0,4])
            c2 = np.hypot(a2,b2)
            
            if c1 > c2:
                lineMerged[0,0] = slope01
                lineMerged[0,1] = x0Start
                lineMerged[0,2] = y0Start
                lineMerged[0,3] = x1End
                lineMerged[0,4] = y1End
                lineMerged[0,5] = c1
            else:
                lineMerged[0,0] = slope01
                lineMerged[0,1] = x1Start
                lineMerged[0,2] = y1Start
                lineMerged[0,3] = x0End
                lineMerged[0,4] = y0End
                lineMerged[0,5] = c2
            
            a = abs(glassLines[2,1]-glassLines[2,3])
            b = abs(glassLines[2,2]-glassLines[2,4])
            c = np.hypot(a,b)
            lineMerged[1,0] = glassLines[2,0]
            lineMerged[1,1] = x2Start
            lineMerged[1,2] = y2Start
            lineMerged[1,3] = x2End
            lineMerged[1,4] = y2End
            lineMerged[1,5] = c
            
        elif (slope02 > angleRangeLower and slope02 < angleRangeUpper):
            a1 = abs(glassLines[0,1]-glassLines[2,3])
            b1 = abs(glassLines[0,2]-glassLines[2,4])
            c1 = np.hypot(a1,b1)
            a2 = abs(glassLines[2,1]-glassLines[0,3])
            b2 = abs(glassLines[2,2]-glassLines[0,4])
            c2 = np.hypot(a2,b2)
            
            if c1 > c2:
                lineMerged[0,0] = slope02
                lineMerged[0,1] = x0Start
                lineMerged[0,2] = y0Start
                lineMerged[0,3] = x2End
                lineMerged[0,4] = y2End
                lineMerged[0,5] = c1
            else:
                lineMerged[0,0] = slope02
                lineMerged[0,1] = x2Start
                lineMerged[0,2] = y2Start
                lineMerged[0,3] = x0End
                lineMerged[0,4] = y0End
                lineMerged[0,5] = c2
            
            a = abs(glassLines[1,1]-glassLines[1,3])
            b = abs(glassLines[1,2]-glassLines[1,4])
            c = np.hypot(a,b)
            lineMerged[1,0] = glassLines[1,0]
            lineMerged[1,1] = x1Start
            lineMerged[1,2] = y1Start
            lineMerged[1,3] = x1End
            lineMerged[1,4] = y1End
            lineMerged[1,5] = c
            
        else:
            a1 = abs(glassLines[1,1]-glassLines[2,3])
            b1 = abs(glassLines[1,2]-glassLines[2,4])
            c1 = np.hypot(a1,b1)
            a2 = abs(glassLines[2,1]-glassLines[1,3])
            b2 = abs(glassLines[2,2]-glassLines[1,4])
            c2 = np.hypot(a2,b2)
            
            if c1 > c2:
                lineMerged[0,0] = slope12
                lineMerged[0,1] = x1Start
                lineMerged[0,2] = y1Start
                lineMerged[0,3] = x2End
                lineMerged[0,4] = y2End
                lineMerged[0,5] = c1
            else:
                lineMerged[0,0] = slope12
                lineMerged[0,1] = x2Start
                lineMerged[0,2] = y2Start
                lineMerged[0,3] = x1End
                lineMerged[0,4] = y1End
                lineMerged[0,5] = c2
            
            a = abs(glassLines[0,1]-glassLines[0,3])
            b = abs(glassLines[0,2]-glassLines[0,4])
            c = np.hypot(a,b)
            lineMerged[1,0] = glassLines[0,0]
            lineMerged[1,1] = x0Start
            lineMerged[1,2] = y0Start
            lineMerged[1,3] = x0End
            lineMerged[1,4] = y0End
            lineMerged[1,5] = c
            
    else: # For 3+ lines
        for i in range(0,len(glassLines)):
            for j in range(i,len(glassLines)):
                if j == i:
                    continue
                a = abs(glassLines[i,1]-glassLines[j,3])
                b = abs(glassLines[i,2]-glassLines[j,4])

                xStart=glassLines[i,1]
                yStart=glassLines[i,2]
                xEnd=glassLines[j,3]
                yEnd=glassLines[j,4]
                
                angleRangeLower = glassLines[i,0]-0.4
                angleRangeUpper = glassLines[i,0]+0.4
                
                slope = linregress([xStart,xEnd],[yStart,yEnd])
                if slope.slope > angleRangeLower and slope.slope < angleRangeUpper:
                    c = np.hypot(a,b)
                    lineMerged[k,0] = slope.slope
                    lineMerged[k,1] = xStart
                    lineMerged[k,2] = yStart
                    lineMerged[k,3] = xEnd
                    lineMerged[k,4] = yEnd
                    lineMerged[k,5] = c
                    k+=1
    return lineMerged

def LinesGrouping(sortedLines):
    # * function that sorts the sorted lines into each glass, again using the slopes
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
    
    # for-loop to determind the range of allowed difference in slope            
    for i in range(0,len(sortedLinesArray)):
        if value is None:
            value=sortedLinesArray[0,0]
            range_upper=value+0.45
        elif range_upper < sortedLinesArray[i,0]:
            value=sortedLinesArray[i,0]
            range_upper=value+0.45
            j+=1
            k=0
        
        # adds the different lines to it's accociated glass. This might have to be changed in the future if more than 5 glasses are introduced in an image. 
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
        k+=1
    lineGroup=glass0
    
    # deletion of zero rows
    for i in range(len(lineGroup)-1,0,-1):
        if ((lineGroup[i,1] == 0) and (lineGroup[i,2] == 0) and (lineGroup[i,3] == 0) and (lineGroup[i,4] == 0)):
            lineGroup = np.delete(lineGroup, (i), axis=0)
    
    return lineGroup

def SortLines(linesP):
    # * function that sorts the found lines after gradient
    slope_sorted=[0 for i in range(len(linesP))]
    for i in range(0, len(linesP)):
        l = linesP[i][0]
        x = [l[0], l[2]]
        y = [l[1], l[3]]
        slope = linregress(x,y)
        # ! dorment function that can correct for full revolutions if this happens. Remember to activate by adding the +/- comments
        if slope.slope > 3.1415:
            slope_fixed = slope.slope#-(2*3.1415)
        elif slope.slope < -3.1415:
            slope_fixed = slope.slope#+(2*3.1415)
        else:
            slope_fixed = slope.slope
        # *               slope         x1   y1     x2   y2
        slope_sorted[i] = [slope_fixed,l[0],l[1],l[2],l[3]]
    slope_sorted.sort(key=lambda x:x[0])
    sortedLines = slope_sorted
    
    return sortedLines

#**********************Main loop***************************

img = cv2.imread('IR_test_cropped.png')

imwidth = img.shape[1]
imheight = img.shape[0]

# ! des_width = int(imwidth * 0.25) this refers to the image scaling in another branch
# ! des_height = int(imheight * 0.25)
des_dim = (imwidth, imheight)
img_screensized = cv2.resize(img, des_dim, interpolation=cv2.INTER_LANCZOS4)

cv2.imshow('image_window',img_screensized)
edges = cv2.Canny(img_screensized, 45, 45)

#houghTeest = HoughLinesSearchSkimage(edges)
edges_hough = HoughLinesSearch(edges)

# ? Add calculations of the distance between lines, and the angle bestween lines. Use this to decide if to lines belong to the same vial.
print("--- %s seconds ---" % (time.time()-start_time))
cv2.imshow('edge_window',edges)
cv2.imshow('edge_hough',edges_hough)
cv2.imwrite('hough_edges.png',edges_hough)
cv2.waitKey(0)
cv2.destroyAllWindows()
