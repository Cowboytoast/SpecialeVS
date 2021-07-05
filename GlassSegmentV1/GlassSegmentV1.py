import cv2
import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt
MassTestData = genfromtxt('MassTest.csv', delimiter=',')

NumberofTests = 300
AllTrue = 0
OrientationFailed = 0
PickupFailed = 0
HandoffFailed = 0
PickupHandoffFalse = 0
OnlyHandoffFalse = 0
OriPickupHandoffFailed = 0
AllFailed = 0

for i in range(0,len(MassTestData)):
    if MassTestData[i,1] == 1.0 and MassTestData[i,2] == 1.0 and MassTestData[i,3] == 1.0 and MassTestData[i,4] == 1.0:
        AllTrue += 1
print("Succes:",AllTrue)

for i in range(0,len(MassTestData)):
    if MassTestData[i,2] == 0:
        OrientationFailed += 1
print("Orientation failed:",OrientationFailed)

for i in range(0,len(MassTestData)):
    if MassTestData[i,3] == 0:
        PickupFailed += 1
print("Pickup failed:",PickupFailed)

for i in range(0,len(MassTestData)):
    if MassTestData[i,4] == 0:
        HandoffFailed += 1
print("Handoff failed:",HandoffFailed)

for i in range(0,len(MassTestData)):
    if MassTestData[i,1] == 1.0 and MassTestData[i,3] == 0 and MassTestData[i,4] == 0:
        PickupHandoffFalse += 1       
print("Both pick-up and handoff failed:",PickupHandoffFalse)

for i in range(0,len(MassTestData)):
    if MassTestData[i,1] == 1.0 and MassTestData[i,3] == 1.0 and MassTestData[i,4] == 0:
        OnlyHandoffFalse += 1
print("Only handoff failed:",OnlyHandoffFalse)
   
for i in range(0,len(MassTestData)):
    if MassTestData[i,1] == 1.0 and MassTestData[i,2] == 0 and MassTestData[i,3] == 0 and MassTestData[i,4] == 0:
        OriPickupHandoffFailed += 1
print("Orientation, Pickup and Handoff failed:",OriPickupHandoffFailed)

for i in range(0,len(MassTestData)):
    if MassTestData[i,1] == 0 and MassTestData[i,2] == 0 and MassTestData[i,3] == 0 and MassTestData[i,4] == 0:
        AllFailed += 1
print("Failed:",AllFailed)


ProsFailed = ((NumberofTests-AllTrue)/NumberofTests)*100
print("Percentage of test failed:",ProsFailed)

