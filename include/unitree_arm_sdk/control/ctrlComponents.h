#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/arm_common.h"
#include "utilities/CSVTool.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "message/udp.h"
#include "utilities/loop.h"

struct CtrlComponents{
public:
    CtrlComponents(double deltaT);
    ~CtrlComponents();
    void sendRecv();
    void armCtrl(Vec6 q, Vec6 qd, Vec6 tau);
    void gripperCtrl(double gripperPos, double gripperW, double gripperTau);

    LowlevelCmd *lowcmd;
    LowlevelState *lowstate;
    double dt;
    SendCmd sendCmd; // command to control the arm
    RecvState recvState; // the arm state receive from udp
    ArmFSMState statePast;
private:
    UDPPort *_udp;
};

#endif