#ifndef __UNITREEARM_H
#define __UNITREEARM_H

#include "control/ctrlComponents.h"

class unitreeArm{
public:
    unitreeArm(CtrlComponents * ctrlComp);
    ~unitreeArm();
    void setFsm(ArmFSMState fsm);
    void backToStart();
    void labelRun(std::string label);
    void labelSave(std::string label);
    void teach(std::string label);
    void teachRepeat(std::string label);
    void calibration();
    void MoveJ(Vec6 posture, double maxSpeed );
    void MoveJ(Vec6 posture, double gripperPos, double maxSpeed);
    void MoveL(Vec6 posture, double maxSpeed);
    void MoveL(Vec6 posture, double gripperPos, double maxSpeed);
    void MoveC(Vec6 middlePosutre, Vec6 endPosture, double maxSpeed);
    void MoveC(Vec6 middlePosutre, Vec6 endPosture, double gripperPos, double maxSpeed);
    void setTraj();
    void startTraj();
    void startTrack(ArmFSMState fsm);

    void sendRecv();

    Vec6 q, qd, tau;
    Vec6 qPast, qdPast, tauPast;
    double gripperQ, gripperW, gripperTau;

    TrajCmd _trajCmd;
    CtrlComponents *_ctrlComp;
    LoopFunc *sendRecvThread;
};

#endif
