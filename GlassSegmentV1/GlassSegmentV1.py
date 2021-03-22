import cv2
import numpy as np
import matplotlib.pyplot as plt

#* Finding candidate based on the distance between the two lines
def PixelWidth(line1, line2, width = 40): #! line1 & line2 should maybe be changed to glassSides instead
    
    candidate=False
    dist = cv2.norm(line1-line2, cv2.NORM_L2)
    
    if dist > width-5 and dist < width+5:
        candidate=True
        
    return candidate

#* Finding candidates based on the length of each side of the glass
def PixelLength(sideA,sideB,length=100,limit=10): #! sideA & sideB should maybe be changed to glassSides instead
    #? How should we do the weigths? With numbers or boolean?
    candidate=1
    
    a = abs(sideA[1,1]-sideA[2,1])
    b = abs(sideA[1,2]-sideA[2,2])
    c = np.hypot(a,b)
    dist_sideA = c
    
    a = abs(sideB[1,1]-sideB[2,1])
    b = abs(sideB[1,2]-sideB[2,2])
    c = np.hypot(a,b)
    dist_sideB = c
    
    if dist_sideA > length-limit and dist_sideA < length-limit and dist_sideB > length-limit and dist_sideB < length+limit:
        candidate *= 1
    elif dist_sideA > length-limit and dist_sideA < length-limit:
        candidate *= 0.7
    elif dist_sideB > length-limit and dist_sideB < length+limit:
        candidate *= 0.7
    elif dist_sideA < length-limit and dist_sideB > length-limit and dist_sideB < length+limit:
        candidate *= 0.5
    elif dist_sideB < length-limit and dist_sideA > length-limit and dist_sideA < length+limit:
        candidate *= 0.5
    else:
        candidate *= 0
    
    return candidate


#cv2.imshow('Matches', img_screensized)
#cv2.imshow('Contour image', edges_lownoise)
cv2.waitKey(0)
cv2.destroyAllWindows()