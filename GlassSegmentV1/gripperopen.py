import socket
import time
import sys
sys.path.append('../config')
sys.path.append('/shome/31383/k383teach/modules')
import robotconfig as rcfg
from src.gripper.class_gripper import Gripper

gripper = Gripper(rcfg.USB_PORT)

gripper.open(wait=True)

gripper.shutdown()



