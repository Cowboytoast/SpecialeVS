import numpy as np

def robotInit():
    
    return 1



def moveCommand(x,y,z,rx,ry,rz,handOffPos):
    #handOffPos = np.empty([6])
    moveComplete = False
    t=robot.transform(x,y,z) #* Generate placement of the glass in robot frame
    s.send('movel(p['+str(t[0])+','+str(t[1])+','+str(t[2])+','+str(rx)+','+str(ry)+','+str(rz)+'],1,0.1)\n')
    robotfunc.wait()
    moveComplete = True
    
    handOffPos[0] = 1
    handOffPos[1] = 2
    handOffPos[2] = 3
    handOffPos[3] = 4
    handOffPos[4] = 5
    handOffPos[5] = 6
    s.send('movel(p['+str(handOffPos[0])+','+str(handOffPos[1])+','+str(handOffPos[2])+','+str(handOffPos[3])+','+str(handOffPos[4])+','+str(handOffPos[5])+'],1,0.1)\n')

    
    return moveComplete