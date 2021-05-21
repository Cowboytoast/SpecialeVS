import sys
import socket
import time
import numpy as np
import cv2
sys.path.append('./modules')
sys.path.append('./modules/src/ur')
sys.path.append('./modules/src/gripper')
sys.path.append('./config')
from modules import robot
from config import robotconfig as rcfg
from modules.src.gripper.class_gripper import Gripper
from modules.src.ur.class_ur import UR as Robot

if  rcfg.grippername=='robotiq': 
  import modules.robotiq as gripper
if  rcfg.grippername=='rg2': 
  import modules.rg2 as gripper

robotfunc = Robot()
gripperfunc = Gripper()
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# TODO: lav init korrekt
# TODO: Lav funktion der kører alle de forskellige funktioner i forbindelse med bevægelse af robot, denne skal så kaldes i main - ingang!


def robotInit():
    #! NEEDS LOT OF FIXING!!!!!!
    HOST1 = rcfg.HOST_IP
    PORT1 = 30003              # The same port as used by the server
    s.connect((HOST1, PORT1))
    
    time.sleep(1)
    
    #! Udfør disse efter alle forbindelser er oprettet osv.
    gripperfunc.open()
    gripperfunc.wait() 
    robot.transform_init([-390.3,350.6, 31.0],[-394, 166.4,31.0],[-245,347,27.6])
    global handOffPos
    handOffPos = handOffPosLOT()
    waitPos()
    
    return

#* Function that works as a "main" function for the robot commands. 
#* Runs a complete cycle with the robot. From getting coordinates to placing the vial and back to start position.
def robotRun(x,y,z,rx,ry,rz,linesFound):
    
    if linesFound == True:
        moveComplete = moveCommand(x,y,z,rx,ry,rz)
        if moveComplete == True:
            handOffComplete = handoffCommand()
        else:
            cv2.waitKey(0) #! lav dette om til et tidsinterval, så som 0.1 sec.
    else:
        cv2.waitKey(0) #! Samme her. Lav noget hvor hvis vi timer-ud så gå tilbage og prøv igen måske.
        
    if handOffComplete ==True:
        takeNewPicture = True
    else:
        takeNewPicture = False
    
    return takeNewPicture


#* Function to move the robot arm from 'waiting' position, to a vial and pick it up. This does not lift it as such, just takes it into the grabber.
def moveCommand(x,y,z,rx,ry,rz):
    moveComplete = False
    t=robot.transform(x,y,z) #* Generate placement of the glass in robot frame
    s.send('movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],1,0.1)\n')
    robotfunc.wait()
    gripper.close(wait=True)
    moveComplete = True
    waitPos()
    
    return moveComplete


#* Function that generates and sends a command to move to handoff placement of vials. Uses an array with x,y,z (for the transform) and rx,ry,rz with the angles.
#* The array are generated in the function 'handOffPosLOT'.
def handoffCommand():
    
    #! handoffpos needs to be adjudsted with 16mm per turn, with a max of 16. If we are smart, lay it out so we only have to adjust in one direction.
    #? Should we make a global counter, or should we make an internal counter that is passed to maintain its value?
    global extractCounter
    extractCounter += 1
    
    handOffComplete = False
    
    handOffPos = handOffPosLOT()
    
    s.send('movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(handOffPos[3])+','+str(handOffPos[4,extractCounter])+','+str(handOffPos[5,extractCounter])+'],1,0.1)\n')
    robotfunc.wait()
    gripper.open(wait=True)
    waitPosComplete = waitPos()
    if waitPosComplete == True:
        handOffComplete == True
    else:
        handOffComplete == False
    
    return handOffComplete


#* Function to generate and move the robot to its waiting position / start-end position
#! Vi kan måske slette denne ved brug af Trajectory Planning (se evt. robot projekt rapport)
def waitPos(x=0,y=0,z=0,rx=0,ry=0,rz=3.14):
    waitPosComplete = False
    t=robot.transform(x,y,z) #* Generate placement of the glass in robot frame
    s.send('movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],1,0.1)\n')
    robotfunc.wait()
    waitPosComplete = True
    return waitPosComplete

#* The function creates a Look-up Table since we have a finite number of specific places. This is utilized to speed up the program such that the positions only have to be calculated
#* at the beginning of the program, thus adding the least amount of time once the recognition and movement of the vials are begun. This function should only be called once
#* after init.
def handOffPosLOT():
    
    handOffPos = np.empty(np.shape(6,16),dtype='object')

    
    for i in range(0,15):
        for j in range(0,2):
            if j == 0 and i == 0: #first iteration of x axis
                handOffPos[i,j] = 1
            elif j == 0:
                handOffPos[i,j] = i * 16 + 1 #every other iteration of x has to be incremented with 16mm, the distance from center of each vial to the next in the handoff tray.
            else:
                handOffPos[i,j] = 0 #y and z axis does not change since we only move the handoff position in one direction
        handOffPos[i,3] = 0 #* rx
        handOffPos[i,4] = 0 #* ry
        handOffPos[i,5] = 0 #* rz
        # handoff angles does not change, thus assigned with constants.
    # moves the coordinates from world frame to robot/base frame. 
    for m in range(0,15):
        t=robot.transform(handOffPos[0,m],handOffPos[1,m],handOffPos[2,m]) #* Generate placement of the glass in robot frame
        handOffPos[0,m] = t[0]
        handOffPos[1,m] = t[1]
        handOffPos[2,m] = t[2]

        #! All numbers above are only as templates as no position are defined yet.
    
    return handOffPos