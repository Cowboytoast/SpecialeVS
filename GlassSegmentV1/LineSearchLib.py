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
    is_nan = False
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
        if math.isnan(slope.slope):
            slope_fixed = 200
            is_nan = True
        # *               slope         x1   y1     x2   y2
        slope_sorted[i] = [slope_fixed,l[0],l[1],l[2],l[3]]
    slope_sorted.sort(key=lambda x:x[0])
    sortedLines = slope_sorted
    
    return sortedLines,is_nan

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

def LineMerge(glassLines,is_nan=False):
    # * function that merge the lines of a side to only one line
    lineMerged = np.zeros([1000,6])
    k = 0
    aloneCnt = 0
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
        if is_nan == True:
            #largest_slope = np.zeros([100,6])
            #m = 0
            rowNumber = np.zeros([3,2])

            for i in range(0,len(glassLines)):
                for j in range(1,len(glassLines)):
                    if i == j:
                        continue
                    else:
                        if abs(glassLines[i,1]-glassLines[j,1]) < 5 and abs(glassLines[i,3]-glassLines[j,3]) < 5:
                            rowNumber[i,:] = [i,j]
                            
            for i in range(0,len(rowNumber)):
                    if not rowNumber[i,1] == 0:
                        lineMerged[k,0] = 50
                        linea = rowNumber[i,0].astype(int)
                        lineb = rowNumber[i,1].astype(int)
                        xCoordinate = glassLines[linea,1]
                        lineMerged[k,1], lineMerged[k,3] = xCoordinate,xCoordinate
                        if glassLines[linea,4] < glassLines[lineb,4] and glassLines[linea,4] < glassLines[lineb,2]:
                            lineMerged[k,4] = glassLines[linea,4]
                        elif glassLines[lineb,4] < glassLines[linea,2] and glassLines[lineb,4] < glassLines[linea,4]:
                            lineMerged[k,4] = glassLines[lineb,4]
                        elif glassLines[linea,2] < glassLines[lineb,2] and glassLines[linea,2] < glassLines[lineb,4]:
                            lineMerged[k,4] = glassLines[linea,2]
                        elif glassLines[lineb,2] < glassLines[linea,2] and glassLines[lineb,2] < glassLines[linea,4]:
                            lineMerged[k,4] = glassLines[lineb,2]
                            
                        if glassLines[linea,4] > glassLines[lineb,4] and glassLines[linea,4] > glassLines[lineb,2]:
                            lineMerged[k,2] = glassLines[linea,4]
                        elif glassLines[lineb,4] > glassLines[linea,4] and glassLines[lineb,4] > glassLines[linea,2]:
                            lineMerged[k,2] = glassLines[lineb,4]
                        elif glassLines[linea,2] > glassLines[lineb,2] and glassLines[linea,2] > glassLines[lineb,4]:
                            lineMerged[k,2] = glassLines[linea,2]
                        else:
                            lineMerged[k,2] = glassLines[lineb,2]
                            
                        a = abs(lineMerged[k,1]-lineMerged[k,3])
                        b = abs(lineMerged[k,2]-lineMerged[k,4])
                        lineMerged[k,5] = np.hypot(a,b)
                        
            lineExist = np.where(rowNumber == 1)
            lineExist = np.array(lineExist)
            isEmpty = lineExist.size == 0
            if isEmpty:
                lineMerged[k+1,0] = 50
                lineMerged[k+1,1] = glassLines[1,1]
                lineMerged[k+1,2] = glassLines[1,2]
                lineMerged[k+1,3] = glassLines[1,3]
                lineMerged[k+1,4] = glassLines[1,4]
                lineMerged[k+1,5] = np.hypot(abs(lineMerged[k,1]-lineMerged[k,3]),abs(lineMerged[k,2]-lineMerged[k,4]))
        
        else:
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
        if is_nan == True:
            #largest_slope = np.zeros([100,6])
            #m = 0

            for i in range(0,len(glassLines)):
                for j in range(1,len(glassLines)):
                    if i == j:
                        continue
                    else:
                        if abs(glassLines[i,1]-glassLines[j,1]) < 5 and abs(glassLines[i,3]-glassLines[j,3]) < 5:
                            lineMerged[k,0] = 50
                            if glassLines[i,1] < glassLines[j,1]:
                                lineMerged[k,1] = glassLines[i,1]
                            else:
                                lineMerged[k,1] = glassLines[j,1]
                            if glassLines[i,2] < glassLines[j,2]:
                                lineMerged[k,2] = glassLines[i,2]
                            else:
                                lineMerged[k,2] = glassLines[j,2]
                            if glassLines[i,3] > glassLines[j,3]:
                                lineMerged[k,3] = glassLines[i,3]
                            else:
                                lineMerged[k,3] = glassLines[j,3]
                            if glassLines[i,4] > glassLines[j,4]:
                                lineMerged[k,4] = glassLines[i,4]
                            else:
                                lineMerged[k,4] = glassLines[j,4]
            
        else:    
            tmp = np.zeros([100,1])    
                
            for i in range(0,len(glassLines)):
                for j in range(0,len(glassLines)):
                    if j == i:
                        continue
                        
                    if (abs(glassLines[i,1]-glassLines[j,1]) > 5 or abs(glassLines[i,2]-glassLines[j,2]) > 5) and (abs(glassLines[i,3]-glassLines[j,3]) > 5 or abs(glassLines[i,4]-glassLines[j,4]) > 5):
                        tmp[i] += 1
                        continue
                    
                    a = abs(glassLines[i,1]-glassLines[j,3])
                    b = abs(glassLines[i,2]-glassLines[j,4])

                    coordinates = np.array(np.r_[glassLines[i,1:3], glassLines[j, 3:5]]) #xstart ystart xend yend

                    angleRangeLower = glassLines[i,0]-angleTolerance
                    angleRangeUpper = glassLines[i,0]+angleTolerance
                    slope = linregress([coordinates[0], coordinates[2]], [coordinates[1], coordinates[3]])
                    slope_fix = slope.slope


                    if is_nan == True:
                        largest_slope[m,0] = slope.slope
                        largest_slope[m,1:5] = coordinates[0:4]
                        m += 1
                        c = np.hypot(a,b)
                        lineMerged[m,5] = c

                    else:
                        if slope_fix > angleRangeLower and slope_fix < angleRangeUpper:
                            c = np.hypot(a,b)
                        
                            lineMerged[k,0] = slope.slope
                            lineMerged[k,1:5] = coordinates
                            lineMerged[k,5] = c
                            k+=1
    '''
            if is_nan == True:
                largest_slope = list(largest_slope)
                #largest_slope.sort(key=lambda x:x[0]) #! CHANGE TO SORT BY ABS VALUE!
                largest_slope = sorted(largest_slope,key=lambda row: np.abs(row[0]))
                largest_slope = np.array(largest_slope)
                lineMerged[0,:] = largest_slope[0,:]
                lineMerged[1,:] = largest_slope[1,:]
                
        singleLines = tmp[~np.all(lineMerged == 0, axis=1)]
        
        for l in range(0,len(singleLines)):
                if tmp[l] == 1:
                    a = abs(glassLines[l,1]-glassLines[l,3])
                    b = abs(glassLines[l,2]-glassLines[l,4])
                    c = np.hypot(a,b)
                    insertArray = np.array([glassLines[l,0],glassLines[l,1],glassLines[l,2],glassLines[l,3],glassLines[l,4],c])
                    np.insert(lineMerged,0,insertArray,axis=0)
              
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
    '''            
    return lineMerged[~np.all(lineMerged == 0, axis=1)]

def HoughLinesSearch(img, houghLength=40, houghDist=5):
    #img has to be the edge detected image.
    #Copy of edge detected image into BGR image for drawing lines.
    paramChange = False
    houghImage = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    #Find HoughLines on the image. Default houghLengt = 40, houghDist=10
    linesP = cv2.HoughLinesP(img, 0.5, np.pi / 225, 50, None, houghLength, houghDist)
        
    if linesP is None or len(linesP) < 0:
        linesP = cv2.HoughLinesP(img, 1, np.pi / 180, 50, None, 30, houghDist)
    else:
        pass
    

    #If-statement drawing lines on the copy, if any lines are found.
    if linesP is not None:
        sortedLines,is_nan = SortLines(linesP)
        LineGrouping = LinesGrouping(sortedLines)
        glassSides = LineMerge(LineGrouping,is_nan)
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

def LineExtend(img, glassSides,lineLength=110):

    if len(glassSides) < 2:
        print("Only one or zero lines found, no extension performed")
        return glassSides

    if glassSides[0,5] == lineLength and glassSides[1,5] == lineLength:
        return glassSides # If both lines are long enough, return

    if glassSides[0, 5] < lineLength and glassSides[1, 5] < lineLength:
        Extends = 2 # Perform 2 extensions
    else:
        Extends = 1 # Perform 1 extension

    for i in range(0, Extends + 1):
        if glassSides[0,5] < glassSides[1,5]:
            lineExtend = 0
            lineKeep = 1
        else:
            lineExtend = 1
            lineKeep = 0

        pointDistStart = math.sqrt((glassSides[lineKeep, 1] - glassSides[lineExtend, 1])**2
                                    + (glassSides[lineKeep, 2] - glassSides[lineExtend, 2])**2)
        pointDistEnd = math.sqrt((glassSides[lineKeep, 3] - glassSides[lineExtend, 3])**2
                                    + (glassSides[lineKeep, 4] - glassSides[lineExtend, 4])**2)

        # Method from https://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point
        v = np.array([glassSides[lineExtend, 3] - glassSides[lineExtend, 1],
                    glassSides[lineExtend, 4] - glassSides[lineExtend, 2]])
        u = np.array([v[0]/(math.sqrt(v[0]**2 + v[1]**2)),v[1]/(math.sqrt(v[0]**2 + v[1]**2))])
        if pointDistStart > pointDistEnd:
            lengthAdd = lineLength * u
            glassSides[lineExtend, 1] = glassSides[lineExtend, 3] - lengthAdd[0]
            glassSides[lineExtend, 2] = glassSides[lineExtend, 4] - lengthAdd[1]
        else:
            lengthAdd = lineLength * u
            glassSides[lineExtend, 3] = glassSides[lineExtend, 1] + lengthAdd[0]
            glassSides[lineExtend, 4] = glassSides[lineExtend, 2] + lengthAdd[1]

        glassSides[lineExtend, 5] = lineLength # set the new hypotenuse

    houghImage = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    b = 255
    g = 0
    r = 0
    for i in range(0, len(glassSides)):
        l = glassSides[i]
        l = l.astype(int)
        cv2.line(houghImage, (l[1], l[2]), (l[3], l[4]), (b,g,r), 1, cv2.LINE_AA)
        g+=-255
        r+=255
        
    cv2.imshow('Lines Extended', houghImage)
    cv2.waitKey(10)
    return glassSides

def grabberPoint(idxs, UpDown, slopes, angle, grabDist = 60):
    # ! Format of sides:
    # * line-pair = |slope1 = a rad | x1start | y1start | x1end | y1end | hyp | slope2 = b rad | x2start | y2start | x2end | y2end | hyp |
    '''
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

    if line_perp[0] > 100: # In case of horizontal lines, threshold at slope of 100
        line_perp[0] = 100

    line_perp[1] = grabPoint[1] - 1 * line_perp[0] * grabPoint[0]
    m = line_perp[0]
    b = line_perp[1]
    x0 = grabPoint[0]
    y0 = grabPoint[1]
    d = lineLength

    x1[0] = 1/(m**2+1)*(-b*m+m*y0-math.sqrt(d**2*m**2-m**2*x0**2-2*b*m*x0+2*m*x0*y0-b**2+2*b*y0+d**2-y0**2)+x0)
    x1[1] = 1/(m**2+1)*(-b*m+m*y0+math.sqrt(d**2*m**2-m**2*x0**2-2*b*m*x0+2*m*x0*y0-b**2+2*b*y0+d**2-y0**2)+x0)
    y1 = m * x1 + b

    dist0 = math.hypot(grabPoint_tmp[0] - x1[0], grabPoint_tmp[1] - y1[0])
    dist1 = math.hypot(grabPoint_tmp[0] - x1[1], grabPoint_tmp[1] - y1[1])
    #dist0 = math.sqrt((grabPoint_tmp[0] - x1[0])**2 + (grabPoint_tmp[1] - y1[0])**2)
    #dist1 = math.sqrt((grabPoint_tmp[0] - x1[1])**2 + (grabPoint_tmp[1] - y1[1])**2)

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
    '''
    #Solution from https://math.stackexchange.com/questions/656500/given-a-point-slope-and-a-distance-along-that-slope-easily-find-a-second-p
    #* Format of idxs: [max_w, max_h, template shape x, template shape y]
    max_w = idxs[0]
    max_h = idxs[1]
    templatex = idxs[2]
    templatey = idxs[3]
    template_center_x = round(max_w + templatex / 2)
    template_center_y = round(max_h + templatey / 2)
    template_slope = np.average(slopes)
    r = math.sqrt(1+template_slope**2)
    # We have four possible orientation combinations so we need four cases
    # Also, two cases in case the angle is close to 0 (vertical glass)
    if angle > 1 and UpDown == 1:
        xgrab = round(template_center_x - grabDist/r)
        ygrab = round(template_center_y - grabDist*template_slope/r)
    elif angle > 1 and UpDown == 0:
        xgrab = round(template_center_x + grabDist/r)
        ygrab = round(template_center_y + grabDist*template_slope/r)
    elif angle <= -1 and UpDown == 1:
        xgrab = round(template_center_x + grabDist/r)
        ygrab = round(template_center_y + grabDist*template_slope/r)
    elif angle <= -1 and UpDown == 0:
        xgrab = round(template_center_x - grabDist/r)
        ygrab = round(template_center_y - grabDist*template_slope/r)
    elif abs(angle) < 1 and UpDown == 1:
        xgrab = round(template_center_x - grabDist/r)
        ygrab = round(template_center_y - grabDist*template_slope/r)
    else:
        xgrab = round(template_center_x + grabDist/r)
        ygrab = round(template_center_y + grabDist*template_slope/r)
    grabPoint = np.array([xgrab, ygrab])
    grabPointAngle = math.radians(angle)
    if UpDown:
        grabPointAngle += math.pi
    
    return grabPoint, grabPointAngle

def templatematch(img, template, houghLocation, h_steps = 30, w_steps = 30, grabDist = 60):
    # * line-pair = |slope1 = a rad | x1start | y1start | x1end | y1end | hyp1 | slope2 = b rad | x2start | y2start | x2end | y2end | hyp2 |

    slopes = np.array([houghLocation[0], houghLocation[6]])
    if (houghLocation[2] + houghLocation[8]) / 2 != (houghLocation[4] + houghLocation[10]) / 2:
        pointsy = min((houghLocation[2] + houghLocation[8]) / 2, (houghLocation[4] + houghLocation[10]) / 2)
        pointidx = np.argmin(np.array([(houghLocation[2] + houghLocation[8]) / 2, (houghLocation[4] + houghLocation[10]) / 2]))
    else:
        pointsy = (houghLocation[2] + houghLocation[8]) / 2
        pointidx = 0
    
    pointsx = np.array([(houghLocation[1] + houghLocation[7]) / 2, (houghLocation[3] + houghLocation[9]) / 2])
    pointsx = pointsx[pointidx]

    max_idx = np.empty([1, 2])
    slope_avg = np.average(slopes)
    angle_offset = math.degrees(math.atan(slope_avg))
    angle_offset = 90 - abs(angle_offset)

    if angle_offset < 0:
        angle_offset += 45
    if slope_avg > 0:
        angle_offset = -angle_offset

    template_rot = imutils.rotate_bound(template, angle_offset)

    Yshifted = pointsy - template_rot.shape[0] / 2
    Xshifted = pointsx - template_rot.shape[1] / 2

    # Do & operation in increments, that is moving the template image a few pixels right/down
    # for each iteration and store most pixel hits
    maxval = 0
    UpDown = 1 # 1 for up, 0 for down
    max_idx = np.zeros((2,1))
    for h in np.arange(int(Yshifted) - int(h_steps/2), int(Yshifted) + int(h_steps/2), 1):
        for w in np.arange(int(Xshifted) - int(w_steps/2), int(Xshifted) + int(w_steps/2), 1):
            if h < 0 or h + template_rot.shape[0] > img.shape[0] or w < 0 or w + template_rot.shape[1] > img.shape[1]:
                break
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            if matches >= maxval:
                maxval = matches
                max_idx = [h, w]
            #rotatingim = np.copy(img)
            #rotatingim[h : h + template_rot.shape[0], w : w + template_rot.shape[1]] = template_rot
            #cv2.imshow('Rotating progress', rotatingim)
            #cv2.waitKey(5)
            
    template_rot = imutils.rotate_bound(template_rot, 180) # Rotate template by 180 deg
    for h in np.arange(int(Yshifted) - int(h_steps/2), int(Yshifted) + int(h_steps/2), 1):
        for w in np.arange(int(Xshifted) - int(w_steps/2), int(Xshifted) + int(w_steps/2), 1):
            if h < 0 or h + template_rot.shape[0] > img.shape[0] or w < 0 or w + template_rot.shape[1] > img.shape[1]:
                break
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            if matches >= maxval:
                maxval = matches
                max_idx = [h, w]
                UpDown = 0
            #rotatingim = np.copy(img)
            #rotatingim[h : h + template_rot.shape[0], w : w + template_rot.shape[1]] = template_rot
            #cv2.imshow('Rotating progress', rotatingim)
            #cv2.waitKey(5)
    template_rot = imutils.rotate_bound(template_rot, 180) # Rotate template by 180 deg

    if (houghLocation[2] + houghLocation[8]) / 2 != (houghLocation[4] + houghLocation[10]) / 2:
        pointsy = max((houghLocation[2] + houghLocation[8]) / 2, (houghLocation[4] + houghLocation[10]) / 2)
        pointidx = np.argmax(np.array([(houghLocation[2] + houghLocation[8]) / 2, (houghLocation[4] + houghLocation[10]) / 2]))
    else:
        pointsy = (houghLocation[2] + houghLocation[8]) / 2
        pointidx = 1

    pointsx = np.array([(houghLocation[1] + houghLocation[7]) / 2, (houghLocation[3] + houghLocation[9]) / 2])
    pointsx = pointsx[pointidx]
    Yshifted = pointsy - template_rot.shape[0] / 2
    Xshifted = pointsx - template_rot.shape[1] / 2

    for h in np.arange(int(Yshifted) + int(h_steps/2), int(Yshifted) - int(h_steps/2), -1):
        for w in np.arange(int(Xshifted) + int(w_steps/2), int(Xshifted) - int(w_steps/2), -1):
            if h < 0 or h + template_rot.shape[0] > img.shape[0] or w < 0 or w > (img.shape[1] + template_rot.shape[1]):
                break
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            if matches >= maxval:
                UpDown = 1 # 1 for up, 0 for down
                maxval = matches
                max_idx = [h, w]
            #rotatingim = np.copy(img)
            #rotatingim[h : h + template_rot.shape[0], w : w + template_rot.shape[1]] = template_rot
            #cv2.imshow('Rotating progress', rotatingim)
            #cv2.waitKey(5)

    template_rot = imutils.rotate_bound(template_rot, 180) # Rotate template by 180 deg

    for h in np.arange(int(Yshifted) + int(h_steps/2), int(Yshifted) - int(h_steps/2), -1):
        for w in np.arange(int(Xshifted) + int(w_steps/2), int(Xshifted) - int(w_steps/2), -1):
            if h < 0 or h + template_rot.shape[0] > img.shape[0] or w < 0 or w + template_rot.shape[1] > img.shape[1]:
                break
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            if matches >= maxval:
                UpDown = 0 # 1 for up, 0 for down
                maxval = matches
                max_idx = [h, w]
            #rotatingim = np.copy(img)
            #rotatingim[h : h + template_rot.shape[0], w : w + template_rot.shape[1]] = template_rot
            #cv2.imshow('Rotating progress', rotatingim)
            #cv2.waitKey(5)

    max_h = int(max_idx[0])
    max_w = int(max_idx[1])
    grabPoint, grabAngle = grabberPoint([max_w, max_h, template_rot.shape[1], template_rot.shape[0]], UpDown, slope_avg, angle_offset)

    # * OVERLAY STUFF***************************************
    final = np.copy(img)
    final = 255 - final
    final = cv2.cvtColor(final,cv2.COLOR_GRAY2RGB)

    TipOutline = cv2.imread('./images/VialTopRed.png')
    # * To overlay template use code below
    if UpDown == 1:
        Overlay = imutils.rotate_bound(TipOutline, angle_offset)
        final[max_h : max_h + Overlay.shape[0],
        max_w : max_w + Overlay.shape[1]] = Overlay
        cv2.putText(final, 'Orientation: Up', (0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    else:
        Overlay = imutils.rotate_bound(TipOutline, angle_offset + 180)
        final[max_h : max_h + Overlay.shape[0],
        max_w : max_w + Overlay.shape[1]] = Overlay
        cv2.putText(final, 'Orientation: Down', (0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.circle(final, (grabPoint[0], grabPoint[1]), 3, color = (0,0,255), thickness=2)
    return final, UpDown, grabPoint, grabAngle

def templatematchOneLine(img, template, houghLocation, h_steps = 30, w_steps = 30, grabDist = 60):
    # * line = |slope1 = a rad | x1start | y1start | x1end | y1end | hyp1
    if houghLocation.size < 11:
        houghLocation = np.append(houghLocation, houghLocation, axis=0)

    slope = houghLocation[0]
    pointsy = houghLocation[2]
    pointsx = houghLocation[1]

    max_idx = np.empty([1, 2])
    angle_offset = math.degrees(math.atan(slope))
    angle_offset = 90 - abs(angle_offset)

    if angle_offset < 0:
        angle_offset += 45
    if slope > 0:
        angle_offset = -angle_offset

    template_rot = imutils.rotate_bound(template, angle_offset)

    Yshifted = pointsy - template_rot.shape[0]
    Xshifted = pointsx - template_rot.shape[1]

    maxval = 0
    UpDown = 1 # 1 for up, 0 for down
    max_idx = np.zeros((2,1))
    for h in np.arange(int(Yshifted) - int(h_steps/2), int(Yshifted) + int(h_steps/2), 1):
        for w in np.arange(int(Xshifted) - int(w_steps/2), int(Xshifted) + int(w_steps/2), 1):
            if h < 0 or h + template_rot.shape[0] > img.shape[0] or w < 0 or w + template_rot.shape[1] > img.shape[1]:
                break
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            if matches >= maxval:
                maxval = matches
                max_idx = [h, w]
            rotatingim = np.copy(img)
            rotatingim[h : h + template_rot.shape[0], w : w + template_rot.shape[1]] = template_rot
            cv2.imshow('Rotating progress', rotatingim)
            cv2.waitKey(5)
            
    template_rot = imutils.rotate_bound(template_rot, 180) # Rotate template by 180 deg
    for h in np.arange(int(Yshifted) - int(h_steps/2), int(Yshifted) + int(h_steps/2), 1):
        for w in np.arange(int(Xshifted) - int(w_steps/2), int(Xshifted) + int(w_steps/2), 1):
            if h < 0 or h + template_rot.shape[0] > img.shape[0] or w < 0 or w + template_rot.shape[1] > img.shape[1]:
                break
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            if matches >= maxval:
                maxval = matches
                max_idx = [h, w]
                UpDown = 0
            #rotatingim = np.copy(img)
            #rotatingim[h : h + template_rot.shape[0], w : w + template_rot.shape[1]] = template_rot
            #cv2.imshow('Rotating progress', rotatingim)
            #cv2.waitKey(5)
    template_rot = imutils.rotate_bound(template_rot, 180) # Rotate template by 180 deg

    if (houghLocation[2] + houghLocation[8]) / 2 != (houghLocation[4] + houghLocation[10]) / 2:
        pointsy = max((houghLocation[2] + houghLocation[8]) / 2, (houghLocation[4] + houghLocation[10]) / 2)
        pointidx = np.argmax(np.array([(houghLocation[2] + houghLocation[8]) / 2, (houghLocation[4] + houghLocation[10]) / 2]))
    else:
        pointsy = (houghLocation[2] + houghLocation[8]) / 2
        pointidx = 1

    pointsx = np.array([(houghLocation[1] + houghLocation[7]) / 2, (houghLocation[3] + houghLocation[9]) / 2])
    pointsx = pointsx[pointidx]
    Yshifted = pointsy - template_rot.shape[0] / 2
    Xshifted = pointsx - template_rot.shape[1] / 2

    for h in np.arange(int(Yshifted) + int(h_steps/2), int(Yshifted) - int(h_steps/2), -1):
        for w in np.arange(int(Xshifted) + int(w_steps/2), int(Xshifted) - int(w_steps/2), -1):
            if h < 0 or h + template_rot.shape[0] > img.shape[0] or w < 0 or w > (img.shape[1] + template_rot.shape[1]):
                break
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            if matches >= maxval:
                UpDown = 1 # 1 for up, 0 for down
                maxval = matches
                max_idx = [h, w]
            #rotatingim = np.copy(img)
            #rotatingim[h : h + template_rot.shape[0], w : w + template_rot.shape[1]] = template_rot
            #cv2.imshow('Rotating progress', rotatingim)
            #cv2.waitKey(5)

    template_rot = imutils.rotate_bound(template_rot, 180) # Rotate template by 180 deg

    for h in np.arange(int(Yshifted) + int(h_steps/2), int(Yshifted) - int(h_steps/2), -1):
        for w in np.arange(int(Xshifted) + int(w_steps/2), int(Xshifted) - int(w_steps/2), -1):
            if h < 0 or h + template_rot.shape[0] > img.shape[0] or w < 0 or w + template_rot.shape[1] > img.shape[1]:
                break
            matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
            matches = np.count_nonzero(matches)
            if matches >= maxval:
                UpDown = 0 # 1 for up, 0 for down
                maxval = matches
                max_idx = [h, w]
            #rotatingim = np.copy(img)
            #rotatingim[h : h + template_rot.shape[0], w : w + template_rot.shape[1]] = template_rot
            #cv2.imshow('Rotating progress', rotatingim)
            #cv2.waitKey(5)

    max_h = int(max_idx[0])
    max_w = int(max_idx[1])
    grabPoint, grabAngle = grabberPoint([max_w, max_h, template_rot.shape[1], template_rot.shape[0]], UpDown, slope_avg, angle_offset)

    # * OVERLAY STUFF***************************************
    final = np.copy(img)
    final = 255 - final
    final = cv2.cvtColor(final,cv2.COLOR_GRAY2RGB)

    TipOutline = cv2.imread('./images/VialTopRed.png')
    # * To overlay template use code below
    if UpDown == 1:
        Overlay = imutils.rotate_bound(TipOutline, angle_offset)
        final[max_h : max_h + Overlay.shape[0],
        max_w : max_w + Overlay.shape[1]] = Overlay
        cv2.putText(final, 'Orientation: Up', (0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    else:
        Overlay = imutils.rotate_bound(TipOutline, angle_offset + 180)
        final[max_h : max_h + Overlay.shape[0],
        max_w : max_w + Overlay.shape[1]] = Overlay
        cv2.putText(final, 'Orientation: Down', (0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.circle(final, (grabPoint[0], grabPoint[1]), 3, color = (0,0,255), thickness=2)
    return final, UpDown, grabPoint, grabAngle


def removeExtras(houghLocation):
    # * line-pair = |slope1 = a rad | x1start | y1start | x1end | y1end | hyp1 | slope2 = b rad | x2start | y2start | x2end | y2end | hyp2 |
    tmp_lines = np.empty([2, 6])
    tmp_dists = 0
    dist_max = 0
    if len(houghLocation) > 2:
        l1keep = 0 # Keep line 1 and 2 as default
        l2keep = 1
        for cnt1 in range(0, len(houghLocation)):
            for cnt2 in range(cnt1, len(houghLocation)):
                if cnt1 == cnt2:
                    continue
                diststart = math.hypot(houghLocation[cnt1, 1] - houghLocation[cnt2, 1], houghLocation[cnt1, 2] - houghLocation[cnt2, 2])
                distend = math.hypot(houghLocation[cnt1, 3] - houghLocation[cnt2, 3], houghLocation[cnt1, 4] - houghLocation[cnt2, 4])
                tmp_dists = abs(diststart + distend)
                if tmp_dists > dist_max:
                    l1keep = cnt1
                    l2keep = cnt2
    else:
        return houghLocation
    tmp_lines[0, :] = houghLocation[l1keep, :]
    tmp_lines[1, :] = houghLocation[l2keep, :]
    return tmp_lines

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