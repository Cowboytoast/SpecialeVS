import socket
import time
import sys
sys.path.append('../config')
#sys.path.append('/shome/31383/h2/room011/grouphuh/modules') ikke sikker på denne linje er nødvendig mere
from modules import robotconfig as rcfg
from modules.src.gripper.class_gripper import Gripper

gripper = Gripper(rcfg.USB_PORT)
gripper.close(wait=True)

gripper.shutdown()



