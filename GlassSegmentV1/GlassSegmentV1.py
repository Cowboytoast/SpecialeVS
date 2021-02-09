import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('IMG_0005_cropped.png')

imwidth = img.shape[1]
imheight = img.shape[0]

des_width = int(imwidth * 0.25)
des_height = int(imheight * 0.25)
des_dim = (des_width, des_height)
img_screensized = cv2.resize(img, des_dim, interpolation=cv2.INTER_LANCZOS4)

cv2.imshow('image_window',img_screensized)
edges = cv2.Canny(img_screensized, 40, 40)
kernel = np.ones((5,5))

edges_closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
edges_opened = cv2.morphologyEx(edges_closed, cv2.MORPH_OPEN, kernel)

cv2.imshow('edge_window',edges)
cv2.waitKey(0)
cv2.destroyAllWindows()