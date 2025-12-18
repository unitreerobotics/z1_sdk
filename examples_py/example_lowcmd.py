import os
import sys
sys.path.append("../lib")
import unitree_arm_interface
import time
import numpy as np

print("Press ctrl+\ to quit process.")

def load_desired_trajectory(filename):
        trajectory_dir = os.path.join(os.path.dirname(__file__), "trajectories")
        file_path = os.path.join(trajectory_dir, f"{filename}.npz")
        if os.path.exists(file_path):
            data = np.load(file_path)
            print(f"[INFO] Loaded trajectory from {trajectory_dir}/{filename}.npz")
            return data["t"], data["q"], data["q_dot"], data["tau"]
        else:
            return None, None, None, None
        
t_interp, q_interp, q_dot_interp, tau_interp = load_desired_trajectory("nominal_trajectory")
dt = t_interp[1]-t_interp[0]

# print(tau_interp.T)

np.set_printoptions(precision=3, suppress=True)
arm = unitree_arm_interface.ArmInterface(hasGripper=True)
armModel = arm._ctrlComp.armModel
arm.setFsmLowcmd()

catching_steps = len(t_interp)
q_init = q_interp[:,0]
q_end = q_interp[:,-1]

warm_up_steps = 1000
lastPos = arm.lowstate.getQ()
targetPos = q_init #forward

for i in range(0, warm_up_steps):
    arm.q = lastPos*(1-i/warm_up_steps) + targetPos*(i/warm_up_steps)# set position
    arm.qd = (targetPos-lastPos)/(warm_up_steps*0.002) # set velocity
    arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
    arm.gripperQ = 0

    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
    arm.sendRecv()# udp connection
    # print(arm.lowstate.getQ())
    time.sleep(arm._ctrlComp.dt)

start_time = time.time()
holding_time = 2
while time.time() - start_time < holding_time:
    arm.q = q_init
    arm.qd = np.array([0,0,0,0,0,0])
    arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
    arm.gripperQ = 0

    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
    arm.sendRecv()# udp connection
    # print(arm.lowstate.getQ())
    time.sleep(dt)

kp = np.array([20, 30, 30, 20, 15, 10])
kd = np.array([2000, 2000, 2000, 2000, 2000, 2000])
catching_start_time = time.time()
for i in range(0, catching_steps):
    step_start_time = time.time()
    arm.q = q_interp[:,i]
    arm.qd = q_dot_interp[:,i]
    # arm.tau = tau_interp[:,i]
    arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
    # tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
    # arm.tau = np.clip(tau, -30, 30)
    q = arm.lowstate.getQ()
    qd = arm.lowstate.getQd()
    tau_eff = 25.6 * kp * (arm.q-q) + 0.0128 * kd * (arm.qd-qd) + arm.tau
    print(tau_eff, arm.lowstate.getTau())
    arm.gripperQ = 0

    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
    arm.sendRecv()# udp connection
    # print(arm.lowstate.getQ())
    time_until_next_step = 10 * dt - (time.time()-step_start_time)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)
print(time.time()-catching_start_time)
print(t_interp[-1])

finish_time = time.time()
holding_time = 5
while time.time() - finish_time < holding_time:
    arm.q = q_end
    arm.qd = np.array([0,0,0,0,0,0])
    arm.tau = armModel.inverseDynamics(arm.q, arm.qd, np.zeros(6), np.zeros(6)) # set torque
    arm.gripperQ = 0

    arm.setArmCmd(arm.q, arm.qd, arm.tau)
    arm.setGripperCmd(arm.gripperQ, arm.gripperQd, arm.gripperTau)
    arm.sendRecv()# udp connection
    # print(arm.lowstate.getQ())
    time.sleep(dt)

arm.loopOn()
arm.backToStart()
arm.loopOff()
