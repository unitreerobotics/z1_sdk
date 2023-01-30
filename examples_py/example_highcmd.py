import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np

np.set_printoptions(precision=3, suppress=True)
hasGripper = True
arm =  unitree_arm_interface.ArmInterface(hasGripper)
armState = unitree_arm_interface.ArmFSMState
arm.loopOn()

# 1. highcmd_basic : armCtrlInJointCtrl
arm.labelRun("forward")
arm.startTrack(armState.JOINTCTRL)
for i in range(0, 1000):
    arm.jointCtrlCmd(np.array([0,0,0,-1,0,0,-1]), 0.5)
    time.sleep(0.002)

# 2. highcmd_basic : armCtrlByFSM
arm.labelRun("forward")
arm.MoveJ(np.array([0.5,0.1,0.1,0.5,-0.2,0.5]), 0.0, 1.0)
arm.MoveL(np.array([0,0,0,0.45,-0.2,0.2]), -1.0, 0.3)
arm.MoveC(np.array([0,0,0,0.45,0,0.4]), np.array([0,0,0,0.45,0.2,0.2]), 0.0, 0.3)

# 3. highcmd_basic : armCtrlInCartesian
arm.labelRun("forward")
arm.startTrack(armState.CARTESIAN)
for i in range(0, 1000):
    arm.cartesianCtrlCmd(np.array([0,0,0,0,0,-1,-1]), 0.3, 0.1)
    time.sleep(0.002)

arm.backToStart()
arm.loopOff()

