import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np
np.set_printoptions(precision=3, suppress=True)

print("Press ctrl+\ to quit process.")

arm =  unitree_arm_interface.ArmInterface(hasGripper=True)
ctrlComp = arm._ctrlComp
udp = unitree_arm_interface.UDPPort(IP = "127.0.0.1", toPort=8071, ownPort=8072)
ctrlComp.udp = udp

armState = unitree_arm_interface.ArmFSMState
arm.loopOn()

# 1. highcmd_basic : armCtrlInJointCtrl
arm.labelRun("forward")
arm.startTrack(armState.JOINTCTRL)
jnt_speed = 1.0
for i in range(0, 1000):
    # dp = directions * speed; include 7 joints
    arm.jointCtrlCmd([0,0,0,-1,0,0,-1], jnt_speed)
    time.sleep(arm._ctrlComp.dt)

# 2. highcmd_basic : armCtrlByFSM
arm.labelRun("forward")
gripper_pos = 0.0
jnt_speed = 2.0
arm.MoveJ([0.5,0.1,0.1,0.5,-0.2,0.5], gripper_pos, jnt_speed)
gripper_pos = -1.0
cartesian_speed = 0.5
arm.MoveL([0,0,0,0.45,-0.2,0.2], gripper_pos, cartesian_speed)
gripper_pos = 0.0
arm.MoveC([0,0,0,0.45,0,0.4], [0,0,0,0.45,0.2,0.2], gripper_pos, cartesian_speed)

# 3. highcmd_basic : armCtrlInCartesian
arm.labelRun("forward")
arm.startTrack(armState.CARTESIAN)
angular_vel = 0.3
linear_vel = 0.3
for i in range(0, 1000):
    arm.cartesianCtrlCmd([0,0,0,0,0,-1,-1], angular_vel, linear_vel)
    time.sleep(arm._ctrlComp.dt)

arm.backToStart()
arm.loopOff()

