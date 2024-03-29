import cv2
import numpy as np
import math
import imutils
from scipy.stats import linregress

#*********** GLOBAL PARAMETERS **************
angleTolerance = 0.07

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
        slope_fixed = slope.slope
        if math.isnan(slope_fixed):
            slope_fixed = 50
            is_nan = True
        # *               slope         x1   y1     x2   y2
        slope_sorted[i] = [slope_fixed,l[0],l[1],l[2],l[3]]
    slope_sorted.sort(key=lambda x:x[0])
    sortedLines = slope_sorted

    return sortedLines,is_nan

def LinesGrouping(sortedLines,is_nan):
    # * function that sorts the sorted lines into each glass, again using the slopes
    value = None
    verticalVial = False
    lineGroup = []
    range_upper = 0
    k = 0
    glass = np.zeros([1200, 5])
    centerLines = np.zeros([100,5])
    edgeLines = np.zeros([100,5])
    sortedLinesArray = np.array(sortedLines)
    xMin = 115
    xMax = 435
    yMin = 115
    yMax = 285
    j = 0
    l = 0
    np.set_printoptions(precision=6,suppress=True)

    for i in range(0,len(sortedLinesArray)):
        if (sortedLinesArray[i,1] > xMin and sortedLinesArray[i,1] < xMax and sortedLinesArray[i,2] > yMin and sortedLinesArray[i,2] < yMax) or (sortedLinesArray[i,3] > xMin and sortedLinesArray[i,3] < xMax and sortedLinesArray[i,4] > yMin and sortedLinesArray[i,4] < yMax):
                centerLines[j,:] = sortedLinesArray[i,:]
                j += 1
        else:
            edgeLines[l,:] = sortedLinesArray[i,:]
            l += 1

    centerLines = centerLines[~np.all(centerLines == 0, axis=1)]
    edgeLines = edgeLines[~np.all(edgeLines == 0, axis=1)]
    if centerLines is not None and len(centerLines) > 0:
    # for-loop to determine the range of allowed difference in slope
        for i in range(0,len(centerLines)):
            if value is None:
                value=centerLines[0,0]
                range_upper=value+angleTolerance
            elif range_upper < centerLines[i,0]:
                break
            glass[k,:] = centerLines[i,:]
            k += 1
        lineGroup=glass
    else:
        # for-loop to determine the range of allowed difference in slope
        for i in range(0,len(edgeLines)):
            if value is None:
                value=edgeLines[0,0]
                range_upper=value+angleTolerance
            elif range_upper < edgeLines[i,0]:
                break
            glass[k,:] = edgeLines[i,:]
            k += 1
        lineGroup=glass

    lineGroup = lineGroup[~np.all(lineGroup == 0, axis=1)]
    if is_nan == True:
        for i in range(0,len(lineGroup)):
            if lineGroup[i,0] == 50:
                verticalVial = True
            else:
                continue

    return lineGroup,verticalVial

def LineMerge(glassLines,is_nan=False):
    # * function that merges the lines of a side to only one line
    lineMerged = np.zeros([1000,6])
    k = 0
    if glassLines.size < 1:
        print("One or no lines found, aborting")
        print("#############################################")
        return None

    elif len(glassLines) == 1:
        a = abs(glassLines[0,1]-glassLines[0,3])
        b = abs(glassLines[0,2]-glassLines[0,4])
        c = np.hypot(a,b)
        lineMerged[0,0:5] = glassLines[0,0:5]
        lineMerged[0,5] = c

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

            lineExist = np.where(rowNumber == 0)
            lineExist = np.array(lineExist)
            isEmpty = lineExist.size == 0
            if isEmpty:
                lineMerged[k+1,0] = 50
                lineMerged[k+1,1] = glassLines[0,1]
                lineMerged[k+1,2] = glassLines[0,2]
                lineMerged[k+1,3] = glassLines[0,3]
                lineMerged[k+1,4] = glassLines[0,4]
                lineMerged[k+1,5] = np.hypot(abs(lineMerged[k+1,1]-lineMerged[k+1,3]),abs(lineMerged[k+1,2]-lineMerged[k+1,4]))

            lineExist = np.where(rowNumber == 1)
            lineExist = np.array(lineExist)
            isEmpty = lineExist.size == 0
            if isEmpty:
                lineMerged[k+1,0] = 50
                lineMerged[k+1,1] = glassLines[1,1]
                lineMerged[k+1,2] = glassLines[1,2]
                lineMerged[k+1,3] = glassLines[1,3]
                lineMerged[k+1,4] = glassLines[1,4]
                lineMerged[k+1,5] = np.hypot(abs(lineMerged[k+1,1]-lineMerged[k+1,3]),abs(lineMerged[k+1,2]-lineMerged[k+1,4]))

            lineExist = np.where(rowNumber == 2)
            lineExist = np.array(lineExist)
            isEmpty = lineExist.size == 0
            if isEmpty:
                lineMerged[k+1,0] = 50
                lineMerged[k+1,1] = glassLines[2,1]
                lineMerged[k+1,2] = glassLines[2,2]
                lineMerged[k+1,3] = glassLines[2,3]
                lineMerged[k+1,4] = glassLines[2,4]
                lineMerged[k+1,5] = np.hypot(abs(lineMerged[k+1,1]-lineMerged[k+1,3]),abs(lineMerged[k+1,2]-lineMerged[k+1,4]))

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
            row = len(glassLines)
            rowNumber = np.zeros([row,2])

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
                        k += 1
            j = k
            for i in range(1,len(glassLines)):
                lineExist = np.where(rowNumber == i)
                lineExist = np.array(lineExist)
                isEmpty = lineExist.size == 0
                if isEmpty:
                    lineMerged[j,0] = 50
                    lineMerged[j,0] = 50
                    lineMerged[j,0] = 50
                    lineMerged[j,1] = glassLines[1,1]
                    lineMerged[j,2] = glassLines[1,2]
                    lineMerged[j,3] = glassLines[1,3]
                    lineMerged[j,4] = glassLines[1,4]
                    lineMerged[j,5] = np.hypot(abs(lineMerged[j,1]-lineMerged[j,3]),abs(lineMerged[j,2]-lineMerged[j,4]))
                    j += 1

        else:
            for i in range(0,len(glassLines)):
                for j in range(i,len(glassLines)):
                    if j == i:
                        continue

                    coordinates = np.array(np.r_[glassLines[i,1:3], glassLines[j, 3:5]]) #xstart ystart xend yend
                    angleRangeLower = glassLines[i,0]-angleTolerance
                    angleRangeUpper = glassLines[i,0]+angleTolerance
                    slope = linregress([coordinates[0], coordinates[2]], [coordinates[1], coordinates[3]])
                    if slope.slope > angleRangeLower and slope.slope < angleRangeUpper:
                        a = abs(glassLines[i,1]-glassLines[j,3])
                        b = abs(glassLines[i,2]-glassLines[j,4])
                        c = np.hypot(a,b)

                        lineMerged[k,0] = slope.slope
                        lineMerged[k,1:5] = coordinates
                        lineMerged[k,5] = c
                        k+=1
    return lineMerged[~np.all(lineMerged == 0, axis=1)]

def HoughLinesSearch(img, houghLength=20, houghDist=5):
    #img has to be the edge detected image.
    #Copy of edge detected image into BGR image for drawing lines.
    houghImage = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    houghImageAll = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
    #Find HoughLines on the image. Default houghLength = 40, houghDist=10

    linesP = cv2.HoughLinesP(img, 1, np.pi / 225, 50, None, houghLength, houghDist)
    #If-statement drawing lines on the copy, if any lines are found.
    if linesP is not None:
        index = 0
        idx = []
        for cnt in range(0, len(linesP)):
            p1 = linesP[cnt,0,0], linesP[cnt,0,1] #x, y start
            p2 = linesP[cnt,0,2], linesP[cnt,0,3] #x, y end
            length = math.hypot(p1[0] - p2[0], p1[1] - p2[1])
            if length > 140:
                idx.append(index)
            index+=1
        linesP = np.delete(linesP, idx, axis=0)

        sortedLines,is_nan = SortLines(linesP)
        LineGrouping,is_nan = LinesGrouping(sortedLines,is_nan)
        glassSides = LineMerge(LineGrouping,is_nan)
        b = 255
        g = 0
        r = 0

        # * Draw lines * #
        try:
            if glassSides == None:
                for i in range(0, len(linesP)): #for all lines: "linesP", for one glass all lines: "LineGrouping"
                    l = linesP[i] # same as above
                    l = l.astype(int)
                    cv2.line(houghImage, (l[0,0], l[0,1]), (l[0,2], l[0,3]), (b,g,r), 3, cv2.LINE_AA)
                    g+=-255
                    r+=255
                    cv2.imshow('Lines', houghImage)
                    cv2.waitKey(5)
                return None, None, None
        except:
            pass


        for i in range(0, len(linesP)):
            l = linesP[i]
            l = l.astype(int)
            cv2.line(houghImageAll, (l[0,0], l[0,1]), (l[0,2], l[0,3]), (b,g,r), 2, cv2.LINE_AA)
            g+=-255
            r+=255
            cv2.waitKey(5)

        for i in range(0, len(glassSides)):
            l = glassSides[i]
            l = l.astype(int)
            cv2.line(houghImage, (l[1], l[2]), (l[3], l[4]), (b,g,r), 2, cv2.LINE_AA)
            g+=-255
            r+=255
            cv2.imshow('Lines', houghImage)
        cv2.waitKey(5)

    else:
        glassSides = None
    #return houghImage
    cv2.imshow('Lines', houghImage)
    cv2.waitKey(5)
    return houghImage, houghImageAll, glassSides

def LineExtend(img, glassSides,lineLength=110):

    if len(glassSides) < 2:
        print("Only one or zero lines found, no extension performed")
        return glassSides

    if glassSides[0,5] >= lineLength and glassSides[1,5] >= lineLength:
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

    # Draw the extended lines
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

def grabberPoint(idxs, UpDown, slopes, angle, grabDist = 53):
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
    if angle > 0 and UpDown == 1:
        xgrab = round(template_center_x - grabDist/r)
        ygrab = round(template_center_y - grabDist*template_slope/r)
    elif angle > 0 and UpDown == 0:
        xgrab = round(template_center_x + grabDist/r)
        ygrab = round(template_center_y + grabDist*template_slope/r)
    elif angle <= 0 and UpDown == 1:
        xgrab = round(template_center_x + grabDist/r)
        ygrab = round(template_center_y + grabDist*template_slope/r)
    elif angle <= 0 and UpDown == 0:
        xgrab = round(template_center_x - grabDist/r)
        ygrab = round(template_center_y - grabDist*template_slope/r)

    grabPoint = np.array([xgrab, ygrab])
    grabPointAngle = math.radians(angle) - math.pi / 2

    # Shift the angle by pi to adjust for gripper angle
    if UpDown:
        grabPointAngle += math.pi

    return grabPoint, grabPointAngle

def templatematch(img, template, houghLocation, h_steps = 10, w_steps = 10):
    # Performs the template matching
    maxval = 0
    Acceptthreshold = 190 # Threshold for "instant" accept
    Rejectthreshold = 140 # Lower threshold such that bad matches are disregarded
    iterations = 1
    UpDown = 1 # 1 for up, 0 for down
    while iterations < 5 and maxval < Acceptthreshold: # Increa
        h_steps = round((h_steps * iterations / 2)) # Set window size according to iterations
        w_steps = round((w_steps * iterations / 2)) # Set window size according to iterations
        if houghLocation.size == 12: # Check if two lines (with 6 parameters each) are found
            slopes = np.array([houghLocation[0], houghLocation[6]])
            if (houghLocation[2] + houghLocation[8]) / 2 != (houghLocation[4] + houghLocation[10]) / 2:
                pointsy = min((houghLocation[2] + houghLocation[8]) / 2, (houghLocation[4] + houghLocation[10]) / 2)
                pointidx = np.argmin(np.array([(houghLocation[2] + houghLocation[8]) / 2, (houghLocation[4] + houghLocation[10]) / 2]))
            else:
                pointsy = (houghLocation[2] + houghLocation[8]) / 2
                pointidx = 0
            pointsx = np.array([(houghLocation[1] + houghLocation[7]) / 2, (houghLocation[3] + houghLocation[9]) / 2])
            pointsx = pointsx[pointidx]
            slope_avg = np.average(slopes)
        else:
            h_steps = round(h_steps * 1.5 * iterations / 2) # Larger window if one line is found
            w_steps = round(w_steps * 1.5 * iterations / 2) # Larger window if one line is found
            slope_avg = houghLocation[0]
            pointsy = houghLocation[2]
            pointsx = houghLocation[1]
        if slope_avg >= 50:
            h_steps = h_steps * 3 # Make the window large if the vial is vertical


        max_idx = np.empty([1, 2])
        angle_offset = math.degrees(math.atan(slope_avg))
        angle_offset = 90 - abs(angle_offset)

        if slope_avg > 0:
            angle_offset = -angle_offset # Inverted y-axis

        template_rot = imutils.rotate_bound(template, angle_offset)

        Yshifted = pointsy - template_rot.shape[0] / 2
        Xshifted = pointsx - template_rot.shape[1] / 2

        # Do & operation in increments, that is moving the template image a few pixels right/down
        # for each iteration and store most pixel hits
        max_idx = np.zeros((2,1))
        for h in np.arange(int(Yshifted) - int(h_steps/2), int(Yshifted) + int(h_steps/2), 1):
            for w in np.arange(int(Xshifted) - int(w_steps/2), int(Xshifted) + int(w_steps/2), 1):
                if h - template_rot.shape[0] < 0 or h + template_rot.shape[0] > img.shape[0] or w - template_rot.shape[1] < 0 or w + template_rot.shape[1] > img.shape[1]:
                    break
                matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
                matches = np.count_nonzero(matches)
                if matches >= maxval:
                    UpDown = 1
                    maxval = matches
                    max_idx = [h, w]

        template_rot = imutils.rotate_bound(template_rot, 180) # Rotate template by 180 deg
        for h in np.arange(int(Yshifted) - int(h_steps/2), int(Yshifted) + int(h_steps/2), 1):
            for w in np.arange(int(Xshifted) - int(w_steps/2), int(Xshifted) + int(w_steps/2), 1):
                if h - template_rot.shape[0] < 0 or h + template_rot.shape[0] > img.shape[0] or w - template_rot.shape[1] < 0 or w + template_rot.shape[1] > img.shape[1]:
                    break
                matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
                matches = np.count_nonzero(matches)
                if matches >= maxval:
                    maxval = matches
                    max_idx = [h, w]
                    UpDown = 0

        template_rot = imutils.rotate_bound(template_rot, 180) # Rotate template by 180 deg
        if houghLocation.size == 12:
            if (houghLocation[2] + houghLocation[8]) / 2 != (houghLocation[4] + houghLocation[10]) / 2:
                pointsy = max((houghLocation[2] + houghLocation[8]) / 2, (houghLocation[4] + houghLocation[10]) / 2)
                pointidx = np.argmax(np.array([(houghLocation[2] + houghLocation[8]) / 2, (houghLocation[4] + houghLocation[10]) / 2]))
            else:
                pointsy = (houghLocation[2] + houghLocation[8]) / 2
                pointidx = 1
            pointsx = np.array([(houghLocation[1] + houghLocation[7]) / 2, (houghLocation[3] + houghLocation[9]) / 2])
            pointsx = pointsx[pointidx]
        else:
            pointsy = houghLocation[4]
            pointsx = houghLocation[3]

        Yshifted = pointsy - template_rot.shape[0] / 2
        Xshifted = pointsx - template_rot.shape[1] / 2

        for h in np.arange(int(Yshifted) - int(h_steps/2), int(Yshifted) + int(h_steps/2), 1):
            for w in np.arange(int(Xshifted) - int(w_steps/2), int(Xshifted) + int(w_steps/2), 1):
                if h - template_rot.shape[0] < 0 or h + template_rot.shape[0] > img.shape[0] or w - template_rot.shape[1] < 0 or w + template_rot.shape[1] > img.shape[1]:
                    break
                matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
                matches = np.count_nonzero(matches)
                if matches >= maxval:
                    UpDown = 1 # 1 for up, 0 for down
                    maxval = matches
                    max_idx = [h, w]

        template_rot = imutils.rotate_bound(template_rot, 180) # Rotate template by 180 deg
        for h in np.arange(int(Yshifted) - int(h_steps/2), int(Yshifted) + int(h_steps/2), 1):
            for w in np.arange(int(Xshifted) - int(w_steps/2), int(Xshifted) + int(w_steps/2), 1):
                if h - template_rot.shape[0] < 0 or h + template_rot.shape[0] > img.shape[0] or w - template_rot.shape[1] < 0 or w + template_rot.shape[1] > img.shape[1]:
                    break
                matches = np.logical_and(img[h : h + template_rot.shape[0], w : w + template_rot.shape[1]], template_rot)
                matches = np.count_nonzero(matches)
                if matches >= maxval:
                    UpDown = 0 # 1 for up, 0 for down
                    maxval = matches
                    max_idx = [h, w]

        iterations += 1
    if maxval < Rejectthreshold:
        print("No suitable match, returning")
        return None, None, None, None
    max_h = int(max_idx[0])
    max_w = int(max_idx[1])
    # Get the real world grabbing points
    grabPoint, grabAngle = grabberPoint([max_w, max_h, template_rot.shape[1], template_rot.shape[0]], UpDown, slope_avg, angle_offset)

    # * OVERLAY * #
    final = np.copy(img)
    final = 255 - final
    final = cv2.cvtColor(final,cv2.COLOR_GRAY2RGB)

    TipOutline = cv2.imread('./templates/VialTopRed.png')
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
    # Removes lines in case that more than two lines are passed
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
    phys_x = 267 #mm
    phys_y = 190 #mm
    imdim_y = imdim[0]
    imdim_x = imdim[1]

    factor_x = phys_x / imdim_x
    factor_y = phys_y / imdim_y

    x = (pickuppoint[0] * factor_x) / 1000
    y = (pickuppoint[1] * factor_y) / 1000

    return x, y