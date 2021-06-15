import sys
import socket
import time
import numpy as np
import cv2
import math

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
    robotfunc.transform_init(p0i = [-383.45, 171.83, 893.33],pxi = [-404.21, -100.2, 893.19], pyi = [-191.8, 159, 896.35])
    robot.transform_init([-383.45, 171.83, 893.33], [-404.21, -100.2, 893.19], [-191.8, 159, 896.35])

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
def pickupCommand(x, y, z = -0.014,rx = 0,ry = 0,rz = 0):
    global s

    waitPos()
    t=robot.transform(x,y,z) #* Generate placement of the glass in robot frame
    cmdstring = 'movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],0.6,0.3)' + '\n'
    s.send(cmdstring.encode())

    [x_robot, y_robot, z_robot, rz_robot] = get_URdata()
    while(not(x_robot >= x - 0.01 and x_robot <= x + 0.01
            and y_robot >= y - 0.01 and y_robot <= y + 0.01
            and z_robot >= z - 0.01 and z_robot <= z + 0.01
            and rz_robot >= rz - 0.03 and rz_robot <= rz + 0.03)):
        [x_robot, y_robot, z_robot, rz_robot] = get_URdata()

    z = 0.0295
    t=robot.transform(x,y,z) #* Generate placement of the glass in robot frame
    cmdstring = 'movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],0.2,0.09)' + '\n'
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

    cmdstring = 'movej(['+str(q_b)+','+str(q_s)+','+str(q_e)+','+str(q_w1)+','+str(q_w2)+','+str(q_w3)+'],1.1,1.3)' + '\n'
    s.send(cmdstring.encode())
    q = get_URdata(True)
    while not(q[0] >= q_b - 0.034 and q[0] <= q_b + 0.034
            and q[1] >= q_s - 0.034 and q[1] <= q_s + 0.034
            and q[2] >= q_e - 0.034 and q[2] <= q_e + 0.034
            and q[3] >= q_w1 - 0.034 and q[3] <= q_w1 + 0.034
            and q[4] >= q_w2 - 0.034 and q[4] <= q_w2 + 0.034
            and q[5] >= q_w3 - 0.034 and q[5] <= q_w3 + 0.034):
        q = get_URdata(True)

    q = handoffPos[:, extractCounter]
    extractCounter += 1

    cmdstring = 'movej(['+str(q[0])+','+str(q[1])+','+str(q[2])+','+str(q[3])+','+str(q[4])+','+str(q[5])+'],1.1,1.3)' + '\n'
    s.send(cmdstring.encode())
    q_robot = get_URdata(True)
    while not(q[0] >= q_robot[0] - 0.034 and q[0] <= q_robot[0] + 0.034
            and q[1] >= q_robot[1] - 0.034 and q[1] <= q_robot[1] + 0.034
            and q[2] >= q_robot[2] - 0.034 and q[2] <= q_robot[2] + 0.034
            and q[3] >= q_robot[3] - 0.034 and q[3] <= q_robot[3] + 0.034
            and q[4] >= q_robot[4] - 0.034 and q[4] <= q_robot[4] + 0.034
            and q[5] >= q_robot[5] - 0.034 and q[5] <= q_robot[5] + 0.034):
        q_robot = get_URdata(True)
    gripperOpen()


#* Function to generate and move the robot to its waiting position / start-end position
def waitPos():
    global s
    q_b = -0.5930274174336976
    q_s = -2.1429696608360045
    q_e = 2.230737222704673
    q_w1 = -1.658563888663564
    q_w2 = 1.5707963267948988
    q_w3 = 1.58
    cmdstring = 'movej(['+str(q_b)+','+str(q_s)+','+str(q_e)+','+str(q_w1)+','+str(q_w2)+','+str(q_w3)+'],1.1,1.2)' + '\n'
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

#* The function creates a Look-up Table since we have a finite number of specific places. This is utilized to speed up the program such that the positions only have to be calculated
#* at the beginning of the program, thus adding the least amount of time once the recognition and movement of the vials are begun. This function should only be called once
#* after init.
def handOffPosLOT():
    handOffPos = np.empty((6, 16),dtype=float)
    #* Extract 1
    handOffPos[0,0] = math.radians(46.52)
    handOffPos[1,0] = math.radians(-124.03)
    handOffPos[2,0] = math.radians(84.33)
    handOffPos[3,0] = math.radians(-50.36)
    handOffPos[4,0] = math.radians(90.07)
    handOffPos[5,0] = math.radians(37.39)
    #* Extract 2
    handOffPos[0,1] = math.radians(50.41)
    handOffPos[1,1] = math.radians(-124.56)
    handOffPos[2,1] = math.radians(84.43)
    handOffPos[3,1] = math.radians(-49.97)
    handOffPos[4,1] = math.radians(90.07)
    handOffPos[5,1] = math.radians(33.24)
    #* Extract 3
    handOffPos[0,2] = math.radians(55.35)
    handOffPos[1,2] = math.radians(-124.56)
    handOffPos[2,2] = math.radians(84.43)
    handOffPos[3,2] = math.radians(-49.91)
    handOffPos[4,2] = math.radians(90.07)
    handOffPos[5,2] = math.radians(28.54)
    #* Extract 4
    handOffPos[0,3] = math.radians(60.53)
    handOffPos[1,3] = math.radians(-124.46)
    handOffPos[2,3] = math.radians(84.42)
    handOffPos[3,3] = math.radians(-50)
    handOffPos[4,3] = math.radians(90.07)
    handOffPos[5,3] = math.radians(23.30)
    #* Extract 5
    handOffPos[0,4] = math.radians(65.79)
    handOffPos[1,4] = math.radians(-124.15)
    handOffPos[2,4] = math.radians(84.35)
    handOffPos[3,4] = math.radians(-50.16)
    handOffPos[4,4] = math.radians(90.07)
    handOffPos[5,4] = math.radians(16.24)
    #* Extract 6
    handOffPos[0,5] = math.radians(72.19)
    handOffPos[1,5] = math.radians(-123.04)
    handOffPos[2,5] = math.radians(84.09)
    handOffPos[3,5] = math.radians(-51.12)
    handOffPos[4,5] = math.radians(90.07)
    handOffPos[5,5] = math.radians(9.72)
    #* Extract 7
    handOffPos[0,6] = math.radians(77.53)
    handOffPos[1,6] = math.radians(-122.42)
    handOffPos[2,6] = math.radians(83.96)
    handOffPos[3,6] = math.radians(-51.52)
    handOffPos[4,6] = math.radians(90.07)
    handOffPos[5,6] = math.radians(4.38)
    #* Extract 8
    handOffPos[0,7] = math.radians(83.08)
    handOffPos[1,7] = math.radians(-121.37)
    handOffPos[2,7] = math.radians(83.66)
    handOffPos[3,7] = math.radians(-52.30)
    handOffPos[4,7] = math.radians(90.07)
    handOffPos[5,7] = math.radians(-1.49)
    #* Extract 9
    handOffPos[0,8] = math.radians(88.38)
    handOffPos[1,8] = math.radians(-120.02)
    handOffPos[2,8] = math.radians(83.25)
    handOffPos[3,8] = math.radians(-53.26)
    handOffPos[4,8] = math.radians(90.07)
    handOffPos[5,8] = math.radians(-6.80)
    #* Extract 10
    handOffPos[0,9] = math.radians(92.68)
    handOffPos[1,9] = math.radians(-119.17)
    handOffPos[2,9] = math.radians(82.97)
    handOffPos[3,9] = math.radians(-53.75)
    handOffPos[4,9] = math.radians(90.07)
    handOffPos[5,9] = math.radians(-10.73)
    #* Extract 11
    handOffPos[0,10] = math.radians(97.38)
    handOffPos[1,10] = math.radians(-117.82)
    handOffPos[2,10] = math.radians(82.48)
    handOffPos[3,10] = math.radians(-54.64)
    handOffPos[4,10] = math.radians(90.07)
    handOffPos[5,10] = math.radians(-13.08)
    #* Extract 12
    handOffPos[0,11] = math.radians(101.21)
    handOffPos[1,11] = math.radians(-116.12)
    handOffPos[2,11] = math.radians(81.78)
    handOffPos[3,11] = math.radians(-55.68)
    handOffPos[4,11] = math.radians(90.07)
    handOffPos[5,11] = math.radians(-16.80)
    #* Extract 13
    handOffPos[0,12] = math.radians(104.72)
    handOffPos[1,12] = math.radians(-114.70)
    handOffPos[2,12] = math.radians(81.16)
    handOffPos[3,12] = math.radians(-56.48)
    handOffPos[4,12] = math.radians(90.07)
    handOffPos[5,12] = math.radians(-20.35)
    #* Extract 14
    handOffPos[0,13] = math.radians(108.22)
    handOffPos[1,13] = math.radians(-113.40)
    handOffPos[2,13] = math.radians(80.55)
    handOffPos[3,13] = math.radians(-57.16)
    handOffPos[4,13] = math.radians(90.07)
    handOffPos[5,13] = math.radians(-25.70)
    #* Extract 15
    handOffPos[0,14] = math.radians(111.39)
    handOffPos[1,14] = math.radians(-111.98)
    handOffPos[2,14] = math.radians(79.84)
    handOffPos[3,14] = math.radians(-57.91)
    handOffPos[4,14] = math.radians(90.07)
    handOffPos[5,14] = math.radians(-29)
    #* Extract 16
    handOffPos[0,15] = math.radians(114.43)
    handOffPos[1,15] = math.radians(-110.33)
    handOffPos[2,15] = math.radians(78.93)
    handOffPos[3,15] = math.radians(-58.65)
    handOffPos[4,15] = math.radians(90.07)
    handOffPos[5,15] = math.radians(-31.94)
    return handOffPos