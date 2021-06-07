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
from modules.src.ur.communication_ur import communication_thread as comm

global robotfunc
global gripperfunc
global s

def robotInit():
    global robotfunc
    global gripperfunc
    global s
    try:
        robotfunc = Robot()
    except TimeoutError:
        robotfunc = None
    try:
        gripperfunc = Gripper()
    except IndexError:
        gripperfunc = None
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    HOST1 = rcfg.HOST_IP
    PORT1 = 30003              # The same port as used by the server
    try:
        s.connect((HOST1, PORT1))
    except TimeoutError:
        print("Could not connect to robot, timeout error")
        return
    time.sleep(1)
    global thread
    thread = comm(ip = HOST1, port = PORT1)
    gripperfunc.activate()
    gripperOpen()
    global handOffPos
    global extractCounter
    global printflag
    printflag = False
    extractCounter = 0
    #! Lumie box points
    robotfunc.transform_init(p0i = [-366.26, 162.16, 892.32],pxi = [-385, -91.7, 895.7], pyi = [-191.75, 148.97, 893.57])
    robot.transform_init([-366.26, 162.16, 892.32], [-385, -91.7, 895.7], [-191.75, 148.97, 893.57])

    handOffPos = handOffPosLOT()
    #s.close()
    print("Going to wait position")
    startPos()

#* Function that works as a "main" function for the robot commands. 
#* Runs a complete cycle with the robot. From getting coordinates to placing the vial and back to start position.
def robotRun(x, y, z = 0, rx = 0, ry = 0, rz = 0):
    global extractCounter
    global printflag
    global handOffPos
    contEmpty = True
    if extractCounter == 16:
        contEmpty = False
        if printflag == False:
            print('Container full, please empty container\n')
            print('Press r when container is emptied, to restart pick-up')
            printflag = True
        l = cv2.waitKey(0)
        if l%256 == 114:
            # press 'r'
            extractCounter = 0
            contEmpty = True
            printflag = False
    if contEmpty == True:
        pickupCommand(x, y, rz = rz)
        handoffCommand(handOffPos)
    return


#* Function to move the robot arm from 'waiting' position, to a vial and pick it up. This does not lift it as such, just takes it into the grabber.
def pickupCommand(x, y, z = -0.02,rx = 0,ry = 0,rz = 0):
    global s

    waitPos()
    t=robot.transform(x,y,z) #* Generate placement of the glass in robot frame
    cmdstring = 'movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],0.6,0.1)' + '\n'
    s.send(cmdstring.encode())

    [x_robot, y_robot, z_robot, rz_robot] = get_URdata()
    while(not(x_robot >= x - 0.01 and x_robot <= x + 0.01
            and y_robot >= y - 0.01 and y_robot <= y + 0.01
            and z_robot >= z - 0.01 and z_robot <= z + 0.01
            and rz_robot >= rz - 0.03 and rz_robot <= rz + 0.03)):
        [x_robot, y_robot, z_robot, rz_robot] = get_URdata()

    z = 0.0295
    t=robot.transform(x,y,z) #* Generate placement of the glass in robot frame
    cmdstring = 'movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],0.2,0.01)' + '\n'
    s.send(cmdstring.encode())

    [x_robot, y_robot, z_robot, rz_robot] = get_URdata()
    while(not(x_robot >= x - 0.003 and x_robot <= x + 0.003 and y_robot >= y - 0.003 and y_robot <= y + 0.003 and z_robot >= z - 0.003 and z_robot <= z + 0.003 and rz_robot >= rz - 0.03 and rz_robot <= rz + 0.03)):
        [x_robot, y_robot, z_robot, rz_robot] = get_URdata()

    gripperClose()
    waitPos()


#* Function that generates and sends a command to move to handoff placement of vials. Uses an array with x,y,z (for the transform) and rx,ry,rz with the angles.
#* The array are generated in the function 'handOffPosLOT'.
def handoffCommand(handoffPos):
    #! handoffpos needs to be adjudsted with 16mm per turn, with a max of 16. If we are smart, lay it out so we only have to adjust in one direction.
    global extractCounter
    global s

    q_b =  0.279633
    q_s = -1.921129
    q_e = 2.10817
    q_w1 = -1.757837
    q_w2 = 1.570796
    q_w3 = 1.291163

    cmdstring = 'movej(['+str(q_b)+','+str(q_s)+','+str(q_e)+','+str(q_w1)+','+str(q_w2)+','+str(q_w3)+'],1.1,0.4)' + '\n'
    s.send(cmdstring.encode())
    q = get_URdata(True)
    while not(q[0] >= q_b - 0.034 and q[0] <= q_b + 0.034
            and q[1] >= q_s - 0.034 and q[1] <= q_s + 0.034
            and q[2] >= q_e - 0.034 and q[2] <= q_e + 0.034
            and q[3] >= q_w1 - 0.034 and q[3] <= q_w1 + 0.034
            and q[4] >= q_w2 - 0.034 and q[4] <= q_w2 + 0.034
            and q[5] >= q_w3 - 0.034 and q[5] <= q_w3 + 0.034):
        q = get_URdata(True)

    x = handoffPos[0, extractCounter]
    y = handoffPos[1, extractCounter]
    z = handoffPos[2, extractCounter]
    rx = handoffPos[3, extractCounter]
    ry = handoffPos[4, extractCounter]
    rz = handoffPos[5, extractCounter]
    extractCounter += 1
    t = robot.transform(x,y,z)

    cmdstring = 'movej(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],1.1,0.7)' + '\n'
    s.send(cmdstring.encode())

    [x_robot, y_robot, z_robot, rz_robot] = get_URdata()
    while(not(x_robot >= x - 0.005 and x_robot <= x + 0.005
            and y_robot >= y - 0.005 and y_robot <= y + 0.005
            and z_robot >= z - 0.005 and z_robot <= z + 0.005
            and rz_robot >= rz - 0.035 and rz_robot <= rz + 0.035)):
        [x_robot, y_robot, z_robot, rz_robot] = get_URdata()

    q = get_URdata(True)
    q_w3 = 0.65 # Rotation value to hand off glasses properly
    cmdstring = 'movej(['+str(q[0])+','+str(q[1])+','+str(q[2])+','+str(q[3])+','+str(q[4])+','+str(q_w3)+'],1.1,0.5)' + '\n'
    s.send(cmdstring.encode())
    q = get_URdata(True)
    while not(q[5] >= q_w3 - 0.024 and q[5] <= q_w3 + 0.024):
        q = get_URdata(True)
    gripperOpen()


#* Function to generate and move the robot to its waiting position / start-end position
def waitPos(x=0.1,y=0.1,z=-0.2,rx=0,ry=0,rz=0):
    global s
    q_b = -0.5930274174336976
    q_s = -2.1429696608360045
    q_e = 2.230737222704673
    q_w1 = -1.658563888663564
    q_w2 = 1.5707963267948988
    q_w3 = 0
    cmdstring = 'movej(['+str(q_b)+','+str(q_s)+','+str(q_e)+','+str(q_w1)+','+str(q_w2)+','+str(q_w3)+'],1.1,0.5)' + '\n'
    s.send(cmdstring.encode())
    q = get_URdata(True)
    while not(q[0] >= q_b - 0.034 and q[0] <= q_b + 0.034
            and q[1] >= q_s - 0.034 and q[1] <= q_s + 0.034
            and q[2] >= q_e - 0.034 and q[2] <= q_e + 0.034
            and q[3] >= q_w1 - 0.034 and q[3] <= q_w1 + 0.034
            and q[4] >= q_w2 - 0.034 and q[4] <= q_w2 + 0.034
            and q[5] >= q_w3 - 0.034 and q[5] <= q_w3 + 0.034):
        q = get_URdata(True)

def startPos():
    global s
    q_b = -1.62
    q_s = -2.1429696608360045
    q_e = 2.230737222704673
    q_w1 = -1.658563888663564
    q_w2 = 1.5707963267948988
    q_w3 = 0
    cmdstring = 'movej(['+str(q_b)+','+str(q_s)+','+str(q_e)+','+str(q_w1)+','+str(q_w2)+','+str(q_w3)+'],1.1,0.5)' + '\n'
    s.send(cmdstring.encode())
    q = get_URdata(True)
    while not(q[0] >= q_b - 0.034 and q[0] <= q_b + 0.034
            and q[1] >= q_s - 0.034 and q[1] <= q_s + 0.034
            and q[2] >= q_e - 0.034 and q[2] <= q_e + 0.034
            and q[3] >= q_w1 - 0.034 and q[3] <= q_w1 + 0.034
            and q[4] >= q_w2 - 0.034 and q[4] <= q_w2 + 0.034
            and q[5] >= q_w3 - 0.034 and q[5] <= q_w3 + 0.034):
        q = get_URdata(True)

#* The function creates a Look-up Table since we have a finite number of specific places. This is utilized to speed up the program such that the positions only have to be calculated
#* at the beginning of the program, thus adding the least amount of time once the recognition and movement of the vials are begun. This function should only be called once
#* after init.
def handOffPosLOT():
    handOffPos = np.empty((6, 16),dtype=float)
    for j in range(0,16):
        handOffPos[0,j] = 0.327
        handOffPos[1,j] = j * 0.016 + 0.362 #every other iteration of y has to be incremented with 16mm, the distance from center of each vial to the next in the handoff tray.
        handOffPos[2,j] = 0.075 #z axis does not change since we only move the handoff position in one direction
        handOffPos[3,j] = 0 #* rx
        handOffPos[4,j] = 0 #* ry
        handOffPos[5,j] = -0.154 #* rz
        # handoff angles does not change, thus assigned with constants.

    return handOffPos

def gripperOpen(pos=180,speed=255,force=10):
    gripperfunc.set(pos,speed,force)
    gripperfunc.wait()
    return

def gripperClose(pos=218,speed=50,force=10):
    gripperfunc.set(pos,speed,force)
    gripperfunc.wait()
    return

def get_URdata(joint_data = False):
    global s
    if s in globals():
        s.close()
        time.sleep(0.1)
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
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((rcfg.HOST_IP, 30003))
        time.sleep(0.1)
        return q
    else:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((rcfg.HOST_IP, 30003))
        time.sleep(0.1)
        return x_robot, y_robot, z_robot, rz_robot
