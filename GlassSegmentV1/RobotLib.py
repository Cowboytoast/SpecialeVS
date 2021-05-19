import socket
import time
import numpy as np
import robot
import robotconfig as rcfg
from robot_class import Robot
from gripper import Gripper
if  rcfg.grippername=='robotiq': 
  import robotiq as gripper
if  rcfg.grippername=='rg2': 
  import rg2 as gripper
robotfunc = Robot()
gripperfunc = Gripper()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


def robotInit():
    #! NEEDS LOT OF FIXING!!!!!!
    robot.transform_init([-390.3,350.6, 31.0],[-394, 166.4,31.0],[-245,347,27.6])
    HOST1 = rcfg.HOST_IP
    PORT1 = 30003              # The same port as used by the server
    ur = UR(HOST1, PORT1)
    gripperfunc = Gripper(rcfg.USB_PORT)
    s.connect((HOST1, PORT1))
    return 1



def moveCommand(x,y,z,rx,ry,rz):
    moveComplete = False
    t=robot.transform(x,y,z) #* Generate placement of the glass in robot frame
    s.send('movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],1,0.1)\n')
    robotfunc.wait()
    gripper.close(wait=True)
    moveComplete = True
    waitPos()
    
    return moveComplete

def handoffCommand(handOffPos):
    #* Generate and send command to move to handoff placement of vials. Uses an array with x,y,z (for the transform) and rx,ry,rz with the angles.
    #? Should the array be generated inside this function, thus having no arguments or should it be done in its own function? But where should we call said function, here or outside it?
    t=robot.transform(handOffPos[0],handOffPos[1],handOffPos[2]) #* Generate placement of the glass in robot frame
    s.send('movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(handOffPos[3])+','+str(handOffPos[4])+','+str(handOffPos[5])+'],1,0.1)\n')
    robotfunc.wait()
    handOffComplete = True
    gripper.open(wait=True)
    waitPos()
    
    return handOffComplete

def waitPos(x=0,y=0,z=0,rx=0,ry=0,rz=3.14):
    #* Generate and move the robot to its waiting position / start-end position
    t=robot.transform(x,y,z) #* Generate placement of the glass in robot frame
    s.send('movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],1,0.1)\n')
    robotfunc.wait()
    return 1