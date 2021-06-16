import cv2
import numpy as np
import CalibrationLib as cb
from skimage.util import img_as_ubyte
from skimage import morphology

def PrepImg(img, corners):
    # * Chain should be:
    # * Crop -> Resize to (H,W = 403, 550) -> Cvt to gray -> ...
    # * Hist. stretch -> Blur (rad. = (3,3), SigmaX = 7) -> ...
    # * Unsharp mask (Size (3,3), amount 1, thresh. .131) -> ...
    # * Laplacian edge (delta = 5) -> Cvt. to UByte -> ...
    # * Threshold @ 35
    #img = img[155:550, 304:902]
    if corners == 0: # In case of no calibration
        corners = np.empty([4, 2], dtype = np.uint32)
        corners[0] = [267, 132]
        corners[1] = [930, 133]
        corners[2] = [270, 607]
        corners[3] = [933, 601]
    img = img[corners[0, 1] + 35:corners[2, 1] - 20, corners[0, 0] + 28:corners[1, 0] - 20]
    cv2.imshow("Cropped image", img)
    cv2.waitKey(5)
    img = ResizeToFit(img, H= 403, W = 550)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.equalizeHist(img)
    img = cv2.GaussianBlur(img, (3,3), 7)
    #img = unsharp_mask(img, kernel_size = (3,3), amount = 1, threshold = 0.131)
    img = cv2.Laplacian(img, ddepth = cv2.CV_8U, delta = 5)
    img = img_as_ubyte(img)
    img = cv2.threshold(img, 20, maxval = 255, type = cv2.THRESH_BINARY)
    img = img[1]

    # Filter using contour area and remove small noise
    cnts = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 5:
            cv2.drawContours(img, [c], -1, (0,0,0), -1)

    return img

# * Histogram stretching function
def HistStretch(img):
    tmp = img.copy()
    maxval = tmp.max()
    minval = tmp.min()
    for height in range(img.shape[0]):
        for width in range(img.shape[1]):
            tmp[height, width] = ((tmp[height, width] - minval) / (maxval - minval)) * 255
    return tmp

# https://stackoverflow.com/questions/4993082/how-can-i-sharpen-an-image-in-opencv
def unsharp_mask(image, kernel_size=(5, 5), sigma=1.0, amount=1.0, threshold=0):
    #*Return a sharpened version of the image, using an unsharp mask
    blurred = cv2.GaussianBlur(image, kernel_size, sigma)
    sharpened = float(amount + 1) * image - float(amount) * blurred
    sharpened = np.maximum(sharpened, np.zeros(sharpened.shape))
    sharpened = np.minimum(sharpened, 255 * np.ones(sharpened.shape))
    sharpened = sharpened.round().astype(np.uint8)
    if threshold > 0:
        low_contrast_mask = np.absolute(image - blurred) < threshold
        np.copyto(sharpened, image, where=low_contrast_mask)
    return sharpened

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