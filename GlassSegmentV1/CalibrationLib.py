import numpy as np
import cv2
from cv2 import aruco

#* Library for calibration between camera and box/world frame
#* Here we use ArUco markers to determind the corners of the box, corresponding to P0, Px and Py needed for the transformation between world frame and robot frame.


