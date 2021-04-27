import socket
import time
import sys
sys.path.append('../config')
#sys.path.append('/shome/31383/k383teach/modules') ikke sikker på denne linje er nødvendig mere
from modules import robotconfig as rcfg
from modules.src.gripper.class_gripper import Gripper

gripper = Gripper(rcfg.USB_PORT)

gripper.open(wait=True)

gripper.shutdown()



