#ifndef __UNITREEARM_H
#define __UNITREEARM_H

#include <iostream>
#include "unitree_arm_sdk/unitree_arm_sdk.h"

Vec6 PosturetoVec6(const Posture p){
    Vec6 posture;
    posture(0) = p.roll;
    posture(1) = p.pitch;
    posture(2) = p.yaw;
    posture(3) = p.x;
    posture(4) = p.y;
    posture(5) = p.z;
    return posture;
}

Posture Vec6toPosture(const Vec6 p){
    Posture posture;
    posture.roll = p(0);
    posture.pitch = p(1);
    posture.yaw = p(2);
    posture.x = p(3);
    posture.y = p(4);
    posture.z = p(5);
    return posture;
}

class unitreeArm{
public:
    unitreeArm();
    ~unitreeArm();
    void UDPSendRecv();
    void InitCmdData(SendCmd& sendCmd);
    void setFsm(ArmFSMState fsm);
    void backToStart();
    void labelRun(std::string label);
    void labelSave(std::string label);
    void MoveJ(Vec6 moveJCmd);
    void MoveJ(Vec6 moveJCmd, double gripperPos);
    void MoveL(Vec6 moveLCmd);
    void MoveC(Vec6 middleP, Vec6 endP);
    void getJointState(JointState* jointstate);
    void getGripperState(Posture& gripperState);
    void setTraj(TrajCmd trajCmd);
    
    RecvState _recvState; // the arm state receive from udp
    JointState _jointState[7]; // 6 joint angles and one gripper angle
    SendCmd _sendCmd; // command to control the arm
    TrajCmd _trajCmd;

private:
    UDPPort _udp;
    float deltaTime = 4000; //us
    LoopFunc *_udpThread;
};

#endif