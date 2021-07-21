import cv2
import numpy as np

def PrepImg(img, corners):
    # * Chain should be:
    # * Crop -> Resize to (H,W = 403, 550) -> Cvt to gray -> ...
    # * Hist. stretch -> Blur (rad. = (3,3), SigmaX = 7) -> ...
    # * Unsharp mask (Size (3,3), amount 1, thresh. .131) -> ...
    # * Laplacian edge (delta = 5) -> Cvt. to UByte -> ...
    # * Threshold @ 35
    # Check if corners is not defined meaning that the system is not calibrated
    try:
        if corners == 0: # In case of no calibration
            corners = np.empty([4, 2], dtype = np.uint32)
            corners[0] = [267, 132]
            corners[1] = [931, 135]
            corners[2] = [268, 608]
            corners[3] = [932, 604]
    except:
        pass
    img_cropped = img[corners[0, 1]:corners[2, 1], corners[0, 0]:corners[1, 0]]
    cv2.imshow("Cropped image", img_cropped)
    img = ResizeToFit(img_cropped, H= 403, W = 550)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.equalizeHist(img)
    img = cv2.GaussianBlur(img, (3,3), 7)
    img = cv2.Laplacian(img, ddepth = cv2.CV_8U, delta = 5)
    img = cv2.threshold(img, 12, maxval = 255, type = cv2.THRESH_BINARY)
    img = img[1]

    # Filter using contour area and remove small noise
    cnts = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 8:
            cv2.drawContours(img, [c], -1, (0,0,0), -1)

    return img_cropped, img

# * Resize image to fit screen while keeping aspect ratio
def ResizeToFit(oriimg, H = 980, W = 1820):
    if(len(oriimg.shape) == 3):
        height, width, depth = oriimg.shape
    else:
        height, width = oriimg.shape
    scaleWidth = float(W)/float(width)
    scaleHeight = float(H)/float(height)

    if scaleHeight>scaleWidth:
        imgScale = scaleWidth
    else:
        imgScale = scaleHeight

    newX,newY = oriimg.shape[1]*imgScale, oriimg.shape[0]*imgScale
    newimg = cv2.resize(oriimg,(int(newX),int(newY)))
    return newimg