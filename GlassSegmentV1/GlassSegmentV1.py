import cv2
import numpy as np
import matplotlib.pyplot as plt

#* Finding candidate based on the distance between the two lines
# TODO: Fix the norm function. It is still not correct.
def PixelWidth(sideA,sideB,width=40): #! line1 & line2 should maybe be changed to glassSides instead
    dist = cv2.norm(sideA-sideB, cv2.NORM_L2) #* NORM_L2 is the euclidian distance.
    
    if dist > width-5 and dist < width+5:
        candidate=1
    elif dist > width-10 and dist < width+10:
        candidate=0.75
    elif dist > width-15 and dist < width+15:
        candidate=0.5
    else:
        candidate=0
        
    return candidate

#* Finding candidates based on the length of each side of the glass
def PixelLength(glassLines,length=100,limit=10):
    #? How should we do the weigths? With numbers or boolean?
    a = abs(glassLines[0,1]-glassLines[0,3])
    b = abs(glassLines[0,2]-glassLines[0,4])
    c = np.hypot(a,b)
    dist_sideA = c
    
    a = abs(glassLines[1,1]-glassLines[1,3])
    b = abs(glassLines[1,2]-glassLines[1,4])
    c = np.hypot(a,b)
    dist_sideB = c

    if dist_sideA > length-limit and dist_sideA < length-limit and dist_sideB > length-limit and dist_sideB < length+limit:
        candidate = 1
    elif dist_sideA > length-limit and dist_sideA < length-limit:
        candidate = 0.75
    elif dist_sideB > length-limit and dist_sideB < length+limit:
        candidate = 0.75
    elif dist_sideA <= length-limit and dist_sideB > length-limit and dist_sideB < length+limit:
        candidate = 0.5
    elif dist_sideB <= length-limit and dist_sideA > length-limit and dist_sideA < length+limit:
        candidate = 0.5
    else:
        candidate = 0
    
    return candidate

#* Function for confirming if two sides (with the same angle) belongs to the same glass.
#! Might be moved to houghsearch() in stead
def PixelDist(glassLines,width=40):
    if abs(glassLines[0,1] - glassLines[0,3]) <= width:
        candidate = 1
    else:
        candidate = 0
    
    return candidate

#* suedo main function for the candidate functions.
def GlassCandidate(glassLines):
    
    
    widthCan = PixelWidth(glassLines,glassLines)
    lengthCan = PixelLength(glassLines)
    distCan = PixelDist(glassLines)
    
    candidate = widthCan*lengthCan*distCan
    return candidate

#cv2.imshow('Matches', img_screensized)
#cv2.imshow('Contour image', edges_lownoise)
cv2.waitKey(0)
cv2.destroyAllWindows()