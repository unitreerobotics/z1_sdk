import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np

np.set_printoptions(precision=3, suppress=True)
hasGripper = True
arm = unitree_arm_interface.ArmInterface(hasGripper)
arm.setFsmLowcmd()

duration = 1000
lastPos = arm.lowstate.getQ()
targetPos = np.array([0.0, 1.5, -1.0, -0.54, 0.0, 0.0]) #forward

for i in range(0, duration):
    arm.q = lastPos*(1-i/duration) + targetPos*(i/duration)# set position
    arm.qd = (targetPos-lastPos)/(duration*0.002) # set velocity
    arm.tau = np.zeros(6) # set torque
    arm.gripperQ = -1*(i/duration)
    arm.sendRecv()# udp connection
    print(arm.lowstate.getQ())
    time.sleep(0.002)

arm.loopOn()
arm.backToStart()
arm.loopOff()
