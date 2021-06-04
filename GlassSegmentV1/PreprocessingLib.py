import cv2
import numpy as np
import CalibrationLib as cb
#from scipy import ndimage
from skimage.util import img_as_ubyte

def PrepImg(img, corners):
    # * Chain should be:
    # * Crop -> Resize to (H,W = 403, 550) -> Cvt to gray -> ...
    # * Hist. stretch -> Blur (rad. = (3,3), SigmaX = 7) -> ...
    # * Unsharp mask (Size (3,3), amount 1, thresh. .131) -> ...
    # * Laplacian edge (delta = 5) -> Cvt. to UByte -> ...
    # * Threshold @ 32
    img_cropped = cb.markerCrop(img, corners)
    #img_cropped = img[28:28+336,327:327+528]
    img_cropped = cv2.rotate(img_cropped, rotateCode = cv2.ROTATE_90_CLOCKWISE)
    cv2.imshow("Cropped image", img_cropped)
    cv2.waitKey(5)
    img_screensized = ResizeToFit(img_cropped, H= 403, W = 550)
    img_gray = cv2.cvtColor(img_screensized, cv2.COLOR_BGR2GRAY)
    img_stretched = cv2.equalizeHist(img_gray)
    img_blur = cv2.GaussianBlur(img_stretched, (3,3), 7)
    img_sharpen = unsharp_mask(img_blur, kernel_size = (3,3), amount = 1, threshold = 0.131)
    img_edges = cv2.Laplacian(img_sharpen, ddepth = cv2.CV_8U, delta = 5)
    img_edges = img_as_ubyte(img_edges)
    img_binary = cv2.threshold(img_edges, 32, maxval = 255, type = cv2.THRESH_BINARY)
    img_binary = img_binary[1]
    
    return img_binary

# * Histogram stretching function
def HistStretch(img):
    tmp = img.copy()
    maxval = tmp.max()
    minval = tmp.min()
    for height in range(img.shape[0]):
        for width in range(img.shape[1]):
            tmp[height, width] = ((tmp[height, width] - minval) / (maxval - minval)) * 255
    return tmp

def DiffOfGauss(image, rad1, rad2):
    size1 = rad1 * 2 + 1
    size2 = rad2 * 2 + 1
    img1 = cv2.GaussianBlur(image, (size1, size1), 0)
    img2 = cv2.GaussianBlur(image, (size2, size2), 0)
    DoG = (img1 - img2)
    return DoG

# https://stackoverflow.com/questions/4993082/how-can-i-sharpen-an-image-in-opencv
def unsharp_mask(image, kernel_size=(5, 5), sigma=1.0, amount=1.0, threshold=0):
    """Return a sharpened version of the image, using an unsharp mask."""
    blurred = cv2.GaussianBlur(image, kernel_size, sigma)
    sharpened = float(amount + 1) * image - float(amount) * blurred
    sharpened = np.maximum(sharpened, np.zeros(sharpened.shape))
    sharpened = np.minimum(sharpened, 255 * np.ones(sharpened.shape))
    sharpened = sharpened.round().astype(np.uint8)
    if threshold > 0:
        low_contrast_mask = np.absolute(image - blurred) < threshold
        np.copyto(sharpened, image, where=low_contrast_mask)
    return sharpened

def image_threshold(image, lower, upper = 255):
   

    tmp = image.copy()

    for height in range(tmp.shape[0]):
        for width in range(tmp.shape[1]):
            if tmp[height, width] < lower:
                tmp[height, width] = 0
            elif tmp[height, width] > upper:
                tmp[height, width] = 0
            else:
                tmp[height, width] = 255
                

    return tmp

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

def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result