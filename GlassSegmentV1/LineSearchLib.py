import cv2
import numpy as np
import math
import imutils
from scipy.stats import linregress

#*********** GLOBAL PARAMETERS **************
angleTolerance = 0.1

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
    
    # for-loop to determine the range of allowed difference in slope            
    for i in range(0,len(sortedLinesArray)):
        if value is None:
            value=sortedLinesArray[0,0]
            range_upper=value+angleTolerance
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
    lineMerged = np.zeros([1000,6])
    k = 0
    if glassLines.size < 2:
        print("One or no lines found, aborting")
        print("#############################################")
        return None
    if len(glassLines) == 2: # check if there exist only 2 lines
        a0 = abs(glassLines[0,1]-glassLines[0,3])
        b0 = abs(glassLines[0,2]-glassLines[0,4])
        a1 = abs(glassLines[1,1]-glassLines[1,3])
        b1 = abs(glassLines[1,2]-glassLines[1,4])
        c0 = np.hypot(a0,b0)
        c1 = np.hypot(a1,b1)
        lineMerged[0,0:5] = glassLines[0,0:5]
        lineMerged[1,0:5] = glassLines[1,0:5]
        lineMerged[0,5] = c0
        lineMerged[1,5] = c1
    elif len(glassLines) == 3: # check if there exist only 3 lines
        angleRangeLower = glassLines[0,0]-angleTolerance
        angleRangeUpper = glassLines[0,0]+angleTolerance

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
                lineMerged[0,0],lineMerged[0,1],lineMerged[0,2],lineMerged[0,3],lineMerged[0,4],lineMerged[0,5] = slope12,x2Start,y2Start,x1End,y1End,c2
            
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
                    lineMerged[k,5] = c
                    k+=1
                    
        for i in range(0,len(glassLines)):
            for j in range(1,len(glassLines)):
                check = False
                if i==j:
                    continue
                elif (lineMerged[i,1] >= (lineMerged[j,1]-3) and lineMerged[i,1] <= lineMerged[j,1]+3) and (lineMerged[i,2] >= (lineMerged[j,2]-3) and lineMerged[i,2] <= (lineMerged[j,2]+3)) and (lineMerged[i,3] >= (lineMerged[j,3]-3) and lineMerged[i,3] <= (lineMerged[j,3]+3)) and (lineMerged[i,4] >= (lineMerged[j,4]-3) and lineMerged[i,4] <= (lineMerged[j,4]+3)):
                        lineMerged = np.delete(lineMerged,(j),axis=0)
                        check = True
                if check == False:
                    for m in range(0,len(glassLines)):
                        for n in range(1,len(glassLines)):
                            if m == n:
                                continue
                            elif ((lineMerged[m,1] >= (lineMerged[n,1]-3) and lineMerged[m,1] <= (lineMerged[n,1]+3) and lineMerged[m,2] >= (lineMerged[n,2]-3) and lineMerged[m,2] <= (lineMerged[n,2]+3)) or (lineMerged[m,3] >= (lineMerged[n,3]-3) and lineMerged[m,3] <= (lineMerged[n,3]+3) and lineMerged[m,4] >= (lineMerged[n,4]-3) and lineMerged[m,4] <= (lineMerged[n,4]+3))):
                                if lineMerged[m,5] > lineMerged[n,5]:
                                    lineDelete = n
                                else:
                                    lineDelete = m
                                lineMerged = np.delete(lineMerged,(lineDelete),axis=0)

    return lineMerged[~np.all(lineMerged == 0, axis=1)]

def HoughLinesSearch(img, houghLength=70, houghDist=10):
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
        if glassSides.all() == None:
            return None
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
            cv2.line(houghImage, (l[1], l[2]), (l[3], l[4]), (b,g,r), 1, cv2.LINE_AA)
            g+=-255
            r+=255

    else:
        glassSides = None
    #return houghImage
    cv2.imshow('Lines', houghImage)
    cv2.waitKey(10)
    return glassSides

def LineExtend(glassSides,lineLength=80):
    line0,line1 = False, False
    if glassSides[0,5]<lineLength:
        xDist0,yDist0,line0 = np.sin(math.atan(glassSides[0,0]))*lineLength, np.cos(math.atan(glassSides[0,0]))*lineLength,True
    else:
        pass
        
    if glassSides[1,5]<lineLength:
        xDist1,yDist1,line1 = np.cos(math.atan(glassSides[1,0]))*lineLength, np.sin(math.atan(glassSides[1,0]))*lineLength,True
        
    else:
        pass   
    
    if abs(glassSides[0,1]-glassSides[1,1]) > abs(glassSides[0,3]-glassSides[1,3]):
        if line0==True:
            glassSides[0,1],glassSides[0,2] = glassSides[0,1]+np.round(xDist0), glassSides[0,2]+np.round(yDist0)
        else:
            pass
        if line1==True:
            glassSides[1,1],glassSides[1,2] = glassSides[1,1]+np.round(xDist1), glassSides[1,2]+np.round(yDist1)
        else:
            pass
            
    elif abs(glassSides[0,1]-glassSides[1,1]) < abs(glassSides[0,3]-glassSides[1,3]):
        if line0==True:
            glassSides[0,3],glassSides[0,4] = glassSides[0,3]+np.round(xDist0), glassSides[0,4]+np.round(yDist0)
        else:
            pass
        if line1==True:
            glassSides[1,3],glassSides[1,4] = glassSides[1,3]+np.round(xDist1), glassSides[1,4]+np.round(yDist1)
        else:
            pass
            
    elif abs(glassSides[0,2]-glassSides[1,2]) > abs(glassSides[0,4]-glassSides[1,4]):
        if line0==True:
            glassSides[0,1],glassSides[0,2] = glassSides[0,1]+np.round(xDist0), glassSides[0,2]+np.round(yDist0)
        else:
            pass
        if line1==True:
            glassSides[1,1],glassSides[1,2] = glassSides[1,1]+np.round(xDist1), glassSides[1,2]+np.round(yDist1)
        else:
            pass
            
    elif abs(glassSides[0,2]-glassSides[1,2]) < abs(glassSides[0,4]-glassSides[1,4]):
        if line0==True:
            glassSides[0,3],glassSides[0,4] = glassSides[0,3]+np.round(xDist0), glassSides[0,4]+np.round(yDist0)
        if line1==True:
            glassSides[1,3],glassSides[1,4] = glassSides[1,3]+np.round(xDist1), glassSides[1,4]+np.round(yDist1)
    else:
        pass
           
    return glassSides

def grabberPoint(glassSides, UpDown, lineLength=22):
    # ! Format of sides:
    # * line-pair = |slope1 = a rad | x1start | y1start | x1end | y1end | hyp | slope2 = b rad | x2start | y2start | x2end | y2end | hyp |
    grabPoint=np.empty([6])
    grabPoint_tmp = np.empty([2])
    line_perp = np.empty([2])
    x1 = np.empty([2])
    y1 = np.empty([2])
    
    
    grabPoint[0] = (glassSides[1] + glassSides[3]) / 2 # l1x
    grabPoint[1] = (glassSides[2] + glassSides[4]) / 2 # l1y
    
    grabPoint_tmp[0] = (glassSides[7] + glassSides[9]) / 2 # l2x
    grabPoint_tmp[1] = (glassSides[8] + glassSides[10]) / 2 # l2y
    line = np.polyfit([glassSides[1], glassSides[3]],[glassSides[2], glassSides[4]], 1)
    line_perp[0] = -1/line[0] # Slope of perpendicular line
    line_perp[1] = grabPoint[1] - 1 * line_perp[0] * grabPoint[0]
    m = line_perp[0]
    b = line_perp[1]
    x0 = grabPoint[0]
    y0 = grabPoint[1]
    d = lineLength
    
    x1[0] = 1/(m**2+1)*(-b*m+m*y0-math.sqrt(d**2*m**2-m**2*x0**2-2*b*m*x0+2*m*x0*y0-b**2+2*b*y0+d**2-y0**2)+x0)
    x1[1] = 1/(m**2+1)*(-b*m+m*y0+math.sqrt(d**2*m**2-m**2*x0**2-2*b*m*x0+2*m*x0*y0-b**2+2*b*y0+d**2-y0**2)+x0)
    y1 = m * x1 + b
    
    dist0 = math.sqrt((grabPoint_tmp[0] - x1[0])**2 + (grabPoint_tmp[1] - y1[0])**2)
    dist1 = math.sqrt((grabPoint_tmp[0] - x1[1])**2 + (grabPoint_tmp[1] - y1[1])**2)
    
    if dist0 < dist1:
        grabPoint[2] = x1[0]
        grabPoint[3] = y1[0]
    else:
        grabPoint[2] = x1[1]
        grabPoint[3] = y1[1]
    

    grabPoint[4] = (grabPoint[0] + grabPoint[2]) / 2
    grabPoint[5] = (grabPoint[1] + grabPoint[3]) / 2
    grabPointAngle = math.atan(m) - math.pi / 2
    if UpDown:
        grabPointAngle += math.pi
    
    return grabPoint, grabPointAngle


def templatematch(img, template, houghLocation, h_steps = 40, w_steps = 40):
    # * line-pair = |slope1 = a rad | x1start | y1start | x1end | y1end | slope2 = b rad | x2start | y2start | x2end | y2end
    if houghLocation.size < 7:
        print("One or no lines found!")
        return None
        
    pointsx = np.array([houghLocation[1], houghLocation[3], houghLocation[7], houghLocation[9]])
    pointsy = np.array([houghLocation[2], houghLocation[4], houghLocation[8], houghLocation[10]])
    slopes = np.array([houghLocation[0], houghLocation[6]])
    
    slope_offset = math.degrees(math.atan(np.average(slopes)))
    slope_offset = 90 - abs(slope_offset)
    if slope_offset < 0:
        slope_offset += 45
    
    if np.average(slopes) > 0:
        slope_offset = -slope_offset
        
    template_rot = imutils.rotate_bound(template, slope_offset)
    startPoint = np.argmin(pointsx + pointsy)
    
    Yshifted = pointsy - int(template_rot.shape[0]/2)
    Xshifted = pointsx - int(template_rot.shape[1]/4)
    
    # Do & operation in increments, that is moving the template image a few pixels right/down
    # for each iteration and store most pixel hits
    maxval = 0
    UpDown = 1 # 1 for up, 0 for down
    max_idx = np.zeros((2,1))
    for h in np.arange(int(Yshifted[startPoint]) - int(h_steps/2), int(Yshifted[startPoint]) + int(h_steps/2), 1):
        for w in np.arange(int(Xshifted[startPoint]) - int(w_steps/2), int(Xshifted[startPoint]) + int(w_steps/2), 1):
            if h < 0 or w < 0 or h > (img.shape[0] - template_rot.shape[0]) or w > (img.shape[1] - template_rot.shape[1]):
                return None
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            if matches >= maxval:
                maxval = matches
                max_idx = np.array([h, w])

    startPoint = np.argmax(pointsx + pointsy)
        
    Xshifted = pointsx - int(template_rot.shape[1])
    template_rot = imutils.rotate_bound(template_rot, 180) # Rotate template by 180 deg
    
    for h in np.arange(int(Yshifted[startPoint]) + int(h_steps/2), int(Yshifted[startPoint]) - int(h_steps/2), -1):
        for w in np.arange(int(Xshifted[startPoint]) + int(w_steps/2), int(Xshifted[startPoint]) - int(w_steps/2), -1):
            if h < 0 or w < 0 or h > (img.shape[0] - template_rot.shape[0]) or w > (img.shape[1] - template_rot.shape[1]):
                return None
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            if matches >= maxval:
                UpDown = 0 # 1 for up, 0 for down
                maxval = matches
                max_idx = np.array([h, w])
    max_h = int(max_idx[0])
    max_w = int(max_idx[1])


    # * OVERLAY STUFF***************************************
    final = np.copy(img)
    final = cv2.cvtColor(final,cv2.COLOR_GRAY2RGB)
    
    TipOutline = cv2.imread('./images/VialTopGreen.png')
    # * To overlay template use code below
    if UpDown == 1:
        Overlay = imutils.rotate_bound(TipOutline, slope_offset)
        final[max_h : max_h + Overlay.shape[0],
        max_w : max_w + Overlay.shape[1]] = Overlay
        cv2.putText(final, 'Orientation: Up', (0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    else:
        Overlay = imutils.rotate_bound(TipOutline, slope_offset + 180)
        final[max_h : max_h + Overlay.shape[0],
        max_w : max_w + Overlay.shape[1]] = Overlay
        cv2.putText(final, 'Orientation: Down', (0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    return final, UpDown


def shiftIdx(array):
    # * Finds first non-zero value in a 2D array or 2D array of arrays
    for H in range(array.shape[0]):
        for W in range(array.shape[1]):
            if array[H, W] > 0:
                return H, W


def pixelstocm(pickuppoint, imdim):
    phys_x = 250 #mm
    phys_y = 174 #mm
    imdim_y = imdim[0]
    imdim_x = imdim[1]
    
    factor_x = phys_x / imdim_x
    factor_y = phys_y / imdim_y
    
    x = (pickuppoint[0] * factor_x) / 1000
    y = (pickuppoint[1] * factor_y) / 1000
    
    return x, y