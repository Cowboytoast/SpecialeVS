import cv2
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import linregress

#*********** GLOBAL PARAMETERS **************
angleTolerance = 0.3

#********************************************

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

def LinesGrouping(sortedLines):
    # * function that sorts the sorted lines into each glass, again using the slopes
    value = None
    lineGroup = []
    range_upper = 0
    k = 0
    glass = np.zeros([1200, 5])
    sortedLinesArray = np.array(sortedLines)
    np.set_printoptions(precision=6,suppress=True)
    
    # for-loop to determind the range of allowed difference in slope            
    for i in range(0,len(sortedLinesArray)):
        if value is None:
            value=sortedLinesArray[0,0]
            range_upper=value+0.45
        elif range_upper < sortedLinesArray[i,0]:
            break
        glass[k, 0:5] = sortedLinesArray[i, 0:5]
        k += 1
    lineGroup=glass
    
    # deletion of zero rows
    for i in range(len(lineGroup)-1,0,-1):
        if ((lineGroup[i,1] == 0) and (lineGroup[i,2] == 0) and (lineGroup[i,3] == 0) and (lineGroup[i,4] == 0)):
            lineGroup = np.delete(lineGroup, (i), axis=0)
    return lineGroup

def LineMerge(glassLines):
    # * function that merge the lines of a side to only one line
    lineMerged = np.zeros([800,5])
    k = 0
    if len(glassLines) == 2: # check if there exist only 2 lines
        a = np.array(abs(glassLines[0:2,1] - glassLines[0:2, 3]))
        #a0 = abs(glassLines[0,1]-glassLines[0,3])
        b0 = abs(glassLines[0,2]-glassLines[0,4])
        #a1 = abs(glassLines[1,1]-glassLines[1,3])
        b1 = abs(glassLines[1,2]-glassLines[1,4])
        c0 = np.hypot(a[0],b0)
        c1 = np.hypot(a[1],b1)
        lineMerged[0,0:5] = glassLines[0,0:5]
        lineMerged[1,0:5] =glassLines[1,0:5]
        lineMerged[0,5] = c0
        lineMerged[1,5] = c1
    elif len(glassLines) == 3: # check if there exist only 3 lines
        angleRangeLower = glassLines[0,0]-0.2
        angleRangeUpper = glassLines[0,0]+0.2

        x0Start,y0Start,x0End,y0End = glassLines[0,1:5]
        x1Start,y1Start,x1End,y1End = glassLines[1,1:5]
        x2Start,y2Start,x2End,y2End = glassLines[2,1:5]

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
                lineMerged[0,0],lineMerged[0,1],lineMerged[0,2],lineMerged[0,3],lineMerged[0,4],lineMerged[0,5] = slope01,x0Start,y0Start,x1End,y1End,c1
            else:
                lineMerged[0,0],lineMerged[0,1],lineMerged[0,2],lineMerged[0,3],lineMerged[0,4],lineMerged[0,5] = slope01,x1Start,y1Start,x0End,y0End,c2
            
            a = abs(glassLines[2,1]-glassLines[2,3])
            b = abs(glassLines[2,2]-glassLines[2,4])
            c = np.hypot(a,b)
            lineMerged[1,0],lineMerged[1,1],lineMerged[1,2],lineMerged[1,3],lineMerged[1,4],lineMerged[1,5] = glassLines[2,0],x2Start,y2Start,x2End,y2End,c

            
        elif (slope02 > angleRangeLower and slope02 < angleRangeUpper):
            a1 = abs(glassLines[0,1]-glassLines[2,3])
            b1 = abs(glassLines[0,2]-glassLines[2,4])
            c1 = np.hypot(a1,b1)
            a2 = abs(glassLines[2,1]-glassLines[0,3])
            b2 = abs(glassLines[2,2]-glassLines[0,4])
            c2 = np.hypot(a2,b2)
            
            if c1 > c2:
                lineMerged[0,0],lineMerged[0,1],lineMerged[0,2],lineMerged[0,3],lineMerged[0,4],lineMerged[0,5] = slope02,x0Start,y0Start,x2End,y2End,c1

            else:
                lineMerged[0,0],lineMerged[0,1],lineMerged[0,2],lineMerged[0,3],lineMerged[0,4],lineMerged[0,5] = slope02,x2Start,y2Start,x0End,y0End,c2
            
            a = abs(glassLines[1,1]-glassLines[1,3])
            b = abs(glassLines[1,2]-glassLines[1,4])
            c = np.hypot(a,b)
            lineMerged[1,0],lineMerged[1,1],lineMerged[1,2],lineMerged[1,3],lineMerged[1,4],lineMerged[1,5] = glassLines[1,0],x1Start,y1Start,x1End,y1End,c
            
        else:
            a1 = abs(glassLines[1,1]-glassLines[2,3])
            b1 = abs(glassLines[1,2]-glassLines[2,4])
            c1 = np.hypot(a1,b1)
            a2 = abs(glassLines[2,1]-glassLines[1,3])
            b2 = abs(glassLines[2,2]-glassLines[1,4])
            c2 = np.hypot(a2,b2)
            
            if c1 > c2:
                lineMerged[0,0],lineMerged[0,1],lineMerged[0,2],lineMerged[0,3],lineMerged[0,4],lineMerged[0,5] = slope12,x1Start,y1Start,x2End,y2End,c1
            else:
                lineMerged[0,0],lineMerged[0,1],lineMerged[0,2],lineMerged[0,3],lineMerged[0,4],lineMerged[0,5] = slope12,x1Start,y1Start,x1End,y1End,c2
            
            a = abs(glassLines[0,1]-glassLines[0,3])
            b = abs(glassLines[0,2]-glassLines[0,4])
            c = np.hypot(a,b)
            lineMerged[1,0],lineMerged[1,1],lineMerged[1,2],lineMerged[1,3],lineMerged[1,4],lineMerged[1,5] = glassLines[0,0],x0Start,y0Start,x0End,y0End,c

            
    else: # For 3+ lines
        for i in range(0,len(glassLines)):
            for j in range(i,len(glassLines)):
                if j == i:
                    continue
                a = abs(glassLines[i,1]-glassLines[j,3])
                b = abs(glassLines[i,2]-glassLines[j,4])

                coordinates = np.array(np.r_[glassLines[i,1:3], glassLines[j, 3:5]]) #xstart ystart xend yend

                angleRangeLower = glassLines[i,0]-angleTolerance
                angleRangeUpper = glassLines[i,0]+angleTolerance
                
                slope = linregress([coordinates[0], coordinates[2]], [coordinates[1], coordinates[3]])
                if slope.slope > angleRangeLower and slope.slope < angleRangeUpper:
                    c = np.hypot(a,b)
                    
                    lineMerged[k,0] = slope.slope
                    lineMerged[k,1:5] = coordinates
                    #lineMerged[k,5] = c
                    k+=1
    return lineMerged[~np.all(lineMerged == 0, axis=1)]

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
        '''
        # * For printing all lines use:
        for i in range(0, len(linesP)): #for all lines: "linesP", for one glass all lines: "LineGrouping"
            l = linesP[i] # same as above
            l = l.astype(int)
            cv2.line(houghImage, (l[0,0], l[0,1]), (l[0,2], l[0,3]), (b,g,r), 3, cv2.LINE_AA)
            g+=-255
            r+=255
        '''
        for i in range(0, len(glassSides)): #for all lines: "linesP", for one glass all lines: "LineGrouping"
            l = glassSides[i] # same as above
            l = l.astype(int)
            cv2.line(houghImage, (l[1], l[2]), (l[3], l[4]), (b,g,r), 3, cv2.LINE_AA)
            g+=-255
            r+=255
        print(len(glassSides),"lines found")
    else:
        print("No lines found")
    #return houghImage
    cv2.imshow('Lines', houghImage)
    cv2.waitKey(10)
    return glassSides

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

def LineExtend(glassSides,lineLength=100):
    line0,line1 = False, False
    if glassSides[0,5]<lineLength:
        xDist,yDist,line0 = np.sin(glassSides[0,0])*lineLength, np.cos(glassSides[0,0])*lineLength,True
        
    elif glassSides[1,5]<lineLength:
        xDist,yDist,line1 = np.cos(glassSides[1,0])*lineLength, np.sin(glassSides[1,0])*lineLength,True
        
    else:
        return glassSides    
    
    if abs(glassSides[0,1]-glassSides[1,1]) > abs(glassSides[0,3]-glassSides[1,3]):
        if line0==True:
            glassSides[0,1],glassSides[0,2] = glassSides[0,1]+np.round(xDist), glassSides[0,2]+np.round(yDist)
        elif line1==True:
            glassSides[1,1],glassSides[1,2] = glassSides[1,1]+np.round(xDist), glassSides[1,2]+np.round(yDist)
    else:
        if line0==True:
            glassSides[0,3],glassSides[0,4] = glassSides[0,3]+np.round(xDist), glassSides[0,4]+np.round(yDist)
        elif line1==True:
            glassSides[1,3],glassSides[1,4] = glassSides[1,3]+np,round(xDist), glassSides[1,4]+np.round(yDist)
           
    return glassSides

def grapperPoint(glassSides):
    grapPoint=[]
    
    grapPoint[0,0],grapPoint[0,1] = abs(glassSides[0,1]-glassSides[0,3]), abs(glassSides[0,2]-glassSides[0,4])
    grapPoint[1,0],grapPoint[1,1] = abs(glassSides[1,1]-glassSides[1,3]), abs(glassSides[1,2]-glassSides[1,4])
    grapPointAngle = glassSides[0,0]
    
    return grapPoint,grapPointAngle