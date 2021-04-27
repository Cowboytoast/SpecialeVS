import socket
import time
import numpy as np
import sys
sys.path.append('../config')
sys.path.append('/shome/31383/k383teach/modules')
import robot
import robotconfig as rcfg


from src.gripper.class_gripper import Gripper
from src.ur.class_ur import UR

try:
    HOST1 = rcfg.HOST_IP
    PORT1 = 30003              # The same port as used by the server

    ur = UR(HOST1, PORT1)
    gripper = Gripper(rcfg.USB_PORT)

    ##########################################################################################
    #
    # Insert your function definitions here 




    gripper.open(wait=True)
    ########################################################################################## 
    # Enter your calibration coordinates here
    # 

	
    # To use the calibration paper use function below:
    #robot.transform_init([-90.77, -312.75, 1010.62], [-350.7, -294.61, 1010.69], [-80.61, -129.38, 1010.44])
	
    # To use Lumie box use funtion below:
    robot.transform_init([-436, 172.46, 906.56], [-457.53, -82.02, 906.27], [-268.78, 160.50, 907.45])  
    # Coordinates in DECIMETERS?
    x = 0.14
    y = 0.085
    z = 0.02
    rx = rcfg.HOME_ANGLE[0]
    ry = rcfg.HOME_ANGLE[1]
    rz = rcfg.HOME_ANGLE[2]

    t = robot.transform(x, y, z)
    ur.send_line('movel(p[' + str(t[0]) + ',' + str(t[1]) + ',' + str(t[2]) + ',' + str(rx) + ',' + str(ry) + ',' + str(rz) + '],1,0.1)\n')
    ur.wait()
    gripper.close(wait=True)

    ur.shutdown()
    gripper.shutdown()
except KeyboardInterrupt:
    ur.shutdown()
    gripper.shutdown()
    sys.exit("KeyboardInterrupt")
