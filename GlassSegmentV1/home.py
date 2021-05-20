import socket
import time
import numpy as np
import sys
sys.path.append('../config')
#sys.path.append('/shome/31383/k383teach/modules') ikke sikker på denne linje er nødvendig mere
from modules import robot
from config import robotconfig as rcfg

from modules.src.ur.class_ur import UR

HOST1 = rcfg.HOST_IP
PORT1 = 30003              # The same port as used by the server

ur = UR(HOST1, PORT1)

x = rcfg.HOME_POS[0]
y = rcfg.HOME_POS[1]
z = rcfg.HOME_POS[2]
rx = rcfg.HOME_ANGLE[0]
ry = rcfg.HOME_ANGLE[1]
rz = rcfg.HOME_ANGLE[2]
ur.send_line('movel(p['+str(x)+','+str(y)+','+str(z)+','+str(rx)+','+str(ry)+','+str(rz)+']' +',1,0.1)\n')
ur.wait()
ur.shutdown()
