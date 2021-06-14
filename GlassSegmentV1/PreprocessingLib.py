import cv2
import numpy as np
import CalibrationLib as cb
from skimage.util import img_as_ubyte

def PrepImg(img, corners):
    # * Chain should be:
    # * Crop -> Resize to (H,W = 403, 550) -> Cvt to gray -> ...
    # * Hist. stretch -> Blur (rad. = (3,3), SigmaX = 7) -> ...
    # * Unsharp mask (Size (3,3), amount 1, thresh. .131) -> ...
    # * Laplacian edge (delta = 5) -> Cvt. to UByte -> ...
    # * Threshold @ 35
    #img_cropped = cb.markerCrop(img, corners)
    #img_cropped = img[155:550, 304:902]
    img_cropped = img[corners[3, 1]:corners[1, 1], corners[3, 0]:corners[2, 0]]
    cv2.imshow("Cropped image", img_cropped)
    cv2.waitKey(5)
    img_screensized = ResizeToFit(img_cropped, H= 403, W = 550)
    img_gray = cv2.cvtColor(img_screensized, cv2.COLOR_BGR2GRAY)
    img_stretched = cv2.equalizeHist(img_gray)
    img_blur = cv2.GaussianBlur(img_stretched, (3,3), 7)
    img_sharpen = unsharp_mask(img_blur, kernel_size = (3,3), amount = 1, threshold = 0.131)
    img_edges = cv2.Laplacian(img_sharpen, ddepth = cv2.CV_8U, delta = 5)
    img_edges = img_as_ubyte(img_edges)
    img_binary = cv2.threshold(img_edges, 35, maxval = 255, type = cv2.THRESH_BINARY)
    img_binary = img_binary[1]

    return img_binary

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