import sys
import socket
import time
import numpy as np
import cv2
import argparse
sys.path.append('./modules')
sys.path.append('./modules/src/ur')
sys.path.append('./modules/src/gripper')
sys.path.append('./config')
from modules import robot
from config import robotconfig as rcfg
from modules.src.gripper.class_gripper import Gripper
from modules.src.ur.class_ur import UR as Robot
from modules.src.ur.communication_ur import communication_thread as comm

try:
    robotfunc = Robot()
except TimeoutError:
    robotfunc = None
try:
    gripperfunc = Gripper()
except IndexError:
    gripperfunc = None
if robotfunc == None and gripperfunc == None:
    print('Robot and gripper not connected!')
elif robotfunc == None:
    print('Robot not connected!')
elif gripperfunc == None:
    print('Gripper not connected!')
      
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


def robotInit():
    HOST1 = rcfg.HOST_IP
    PORT1 = 30003              # The same port as used by the server
    s.connect((HOST1, PORT1))
    time.sleep(1)
    global thread
    thread = comm(ip = HOST1, port = PORT1)
    gripperOpen() 
    global handOffPos
    global extractCounter
    extractCounter = 0
    robotfunc.transform_init(p0i = [-70.73,-300.5,1002.39],pxi = [-325.82,-280.86,1002.65], pyi = [-60.36,-119.87,1005.88])
    robot.transform_init([-70.73,-300.5,1002.39],[-325.82,-280.86,1002.65],[-60.36,-119.87,1005.88])
    #handOffPos = handOffPosLOT()
    s.close()
    waitPos()
    return

#* Function that works as a "main" function for the robot commands. 
#* Runs a complete cycle with the robot. From getting coordinates to placing the vial and back to start position.
def robotRun(x = 0,y = 0,z = 0,rx = 0,ry = 0,rz = 0):
    global extractCounter
    contEmpty = True
    if extractCounter == 5:
        contEmpty = False
        print('Container full, please empty container\n')
        print('Press r when container is emptied, to restart pick-up')
        l = cv2.waitKey(1)
        if l%256 == 114:
            # press 'r'
            extractCounter = 0
            contEmpty = True
    if contEmpty == True:
        pickupCommand()
        handoffCommand()
    return 


#* Function to move the robot arm from 'waiting' position, to a vial and pick it up. This does not lift it as such, just takes it into the grabber.
def pickupCommand(x = 0.1,y = 0.1,z = 0.02,rx = 0,ry = 0,rz = 0):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((rcfg.HOST_IP,30003))
    x = 0.1
    y = 0.1
    z = 0.02
    rx = 0
    ry = 0
    rz = 0
    t=robot.transform(x,y,z) #* Generate placement of the glass in robot frame
    cmdstring = 'movej(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],1.1,0.9)' + '\n'
    s.send(cmdstring.encode())
    time.sleep(4)
    cmdstring = 'movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],0.6,0.2)' + '\n'
    s.send(cmdstring.encode())
    s.close()
    [x_robot, y_robot, z_robot, rz_robot] = get_URdata()
    while(not(x_robot >= x - 0.01 and x_robot <= x + 0.01 and y_robot >= y - 0.01 and y_robot <= y + 0.01 and z_robot >= z - 0.01 and z_robot <= z + 0.01 and rz_robot >= rz - 0.09 and rz_robot <= rz + 0.09)):
        [x_robot, y_robot, z_robot, rz_robot] = get_URdata()

    gripperClose()
    waitPos()


#* Function that generates and sends a command to move to handoff placement of vials. Uses an array with x,y,z (for the transform) and rx,ry,rz with the angles.
#* The array are generated in the function 'handOffPosLOT'.
def handoffCommand():
    
    #! handoffpos needs to be adjudsted with 16mm per turn, with a max of 16. If we are smart, lay it out so we only have to adjust in one direction.
    #? Should we make a global counter, or should we make an internal counter that is passed to maintain its value?
    global extractCounter
    extractCounter += 1
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((rcfg.HOST_IP,30003))
    x = -0.27
    y = 0.07
    z = 0.02
    rx = 0
    ry = 0
    rz = 0
    t = robot.transform(x,y,z)
    #handOffPos = handOffPosLOT()

    cmdstring = 'movej(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],1.1,0.9)' + '\n'
    s.send(cmdstring.encode())
    time.sleep(4)
    cmdstring = 'movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],0.6,0.2)' + '\n'
    s.send(cmdstring.encode())
    s.close()
    [x_robot, y_robot, z_robot, rz_robot] = get_URdata()
    while(not(x_robot >= x - 0.01 and x_robot <= x + 0.01 and y_robot >= y - 0.01 and y_robot <= y + 0.01 and z_robot >= z - 0.01 and z_robot <= z + 0.01 and rz_robot >= rz - 0.09 and rz_robot <= rz + 0.09)):
        [x_robot, y_robot, z_robot, rz_robot] = get_URdata()

    gripperOpen()
    waitPos()


#* Function to generate and move the robot to its waiting position / start-end position
#! Vi kan måske slette denne ved brug af Trajectory Planning (se evt. robot projekt rapport)
def waitPos(x=-0.05,y=0.15,z=0.3,rx=0,ry=0,rz=0):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((rcfg.HOST_IP,30003))
    t=robot.transform(x,y,z) #* Generate placement of the glass in robot frame
    time.sleep(0.2)
    cmdstring = 'movej(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],1.1,0.9)' + '\n'
    s.send(cmdstring.encode())
    time.sleep(4)
    q = get_URdata(True)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((rcfg.HOST_IP,30003))
    cmdstring = 'movej(['+str(q[0])+','+str(q[1])+','+str(q[2])+','+str(q[3])+','+str(q[4])+','+str(0)+'],1.1,0.8)' + '\n'
    s.send(cmdstring.encode())
    s.close()
    [x_robot, y_robot, z_robot, _] = get_URdata()
    q = get_URdata(True)
    while(not(x_robot >= x - 0.01 and x_robot <= x + 0.01 and y_robot >= y - 0.01 and y_robot <= y + 0.01 and z_robot >= z - 0.01 and z_robot <= z + 0.01 and q[5] >= -0.1 and q[5] <= 0.1)):
        [x_robot, y_robot, z_robot, _] = get_URdata()

#* The function creates a Look-up Table since we have a finite number of specific places. This is utilized to speed up the program such that the positions only have to be calculated
#* at the beginning of the program, thus adding the least amount of time once the recognition and movement of the vials are begun. This function should only be called once
#* after init.
def handOffPosLOT():
    handOffPos = np.empty((16,6),dtype=float)
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

def gripperOpen(pos=100,speed=255,force=10):
    
    gripperfunc.set(pos,speed,force)
    gripperfunc.open()
    gripperfunc.wait()
    
    return

def gripperClose(pos=20,speed=255,force=10):
    
    gripperfunc.set(pos,speed,force)
    gripperfunc.close()
    gripperfunc.wait()
    
    return
def get_URdata(joint_data = False):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((rcfg.HOST_IP, 30003))
    time.sleep(0.1)
    data = s.recv(1024)
    x_robot = thread.transform_data_point(data = data, data_name = 'x')
    y_robot = thread.transform_data_point(data = data, data_name = 'y')
    z_robot = thread.transform_data_point(data = data, data_name = 'z')
    rz_robot = thread.transform_data_point(data = data, data_name = 'rz')
    q_b = thread.transform_data_point(data = data, data_name = 'q_b')
    q_s = thread.transform_data_point(data = data, data_name = 'q_s')
    q_e = thread.transform_data_point(data = data, data_name = 'q_e')
    q_w1 = thread.transform_data_point(data = data, data_name = 'q_w1')
    q_w2 = thread.transform_data_point(data = data, data_name = 'q_w2')
    q_w3 = thread.transform_data_point(data = data, data_name = 'q_w3')
    q = np.array([q_b, q_s, q_e, q_w1, q_w2, q_w3])
    s.close()
    time.sleep(0.1)
    [x_robot, y_robot, z_robot] = robotfunc.inverse_transform(x = x_robot, y = y_robot, z = z_robot)
    if joint_data == True:
        return q
    else:
        return x_robot, y_robot, z_robot, rz_robot
