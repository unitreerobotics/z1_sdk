#include "control/ctrlComponents.h"

CtrlComponents::CtrlComponents(double deltaT){
    dt = deltaT;
    sendCmd = {0};
    sendCmd.head[0] = 0xFE;
    sendCmd.head[1] = 0xFF;

    _udp = new UDPPort("127.0.0.1", 8071, 8072, RECVSTATE_LENGTH, BlockYN::NO, 500000);
    lowcmd = new LowlevelCmd();
    lowstate = new LowlevelState();
}

CtrlComponents::~CtrlComponents(){
    delete _udp;
    delete lowcmd;
    delete lowstate;
}

void CtrlComponents::sendRecv(){
    if(sendCmd.state == ArmFSMState::LOWCMD){
        for(int i(0); i<7; i++){
            sendCmd.valueUnion.jointCmd[i].Pos  = lowcmd->q.at(i);
            sendCmd.valueUnion.jointCmd[i].W    = lowcmd->dq.at(i);
            sendCmd.valueUnion.jointCmd[i].T    = lowcmd->tau.at(i);
            sendCmd.valueUnion.jointCmd[i].K_P  = lowcmd->kp.at(i);
            sendCmd.valueUnion.jointCmd[i].K_W  = lowcmd->kd.at(i);
        }
    }
    if(sendCmd.track){
        if(recvState.state == ArmFSMState::JOINTCTRL){
            for(int i(0); i<7; i++){//the arm will track joint cmd (Only q and dq)
                sendCmd.valueUnion.jointCmd[i].Pos  = lowcmd->q.at(i);
                sendCmd.valueUnion.jointCmd[i].W    = lowcmd->dq.at(i);
            }
        }else if(recvState.state == ArmFSMState::CARTESIAN){
            // the arm will track posture[0] cmd
            sendCmd.valueUnion.trajCmd.posture[0] = Vec6toPosture(lowcmd->endPosture);
            sendCmd.valueUnion.trajCmd.gripperPos = lowcmd->getGripperQ();
        }
    }
    _udp->send((uint8_t*)&sendCmd, SENDCMD_LENGTH);
    statePast = recvState.state;
    _udp->recv((uint8_t*)&recvState, RECVSTATE_LENGTH);

    for(int i(0); i<7; i++){
        lowstate->q.at(i)   = recvState.jointState[i].Pos;
        lowstate->dq.at(i)  = recvState.jointState[i].W;
        lowstate->ddq.at(i) = recvState.jointState[i].Acc;
        lowstate->tau.at(i) = recvState.jointState[i].T;
    }
    lowstate->endPosture = PosturetoVec6(recvState.cartesianState);
    //motor 0
    lowstate->temperature.at(0)      = recvState.jointState[0].state[0].temperature;
    lowstate->errorstate.at(0)       = recvState.jointState[0].state[0].error;
    lowstate->isMotorConnected.at(0) = recvState.jointState[0].state[0].isConnected.state;
    //motor 1
    lowstate->temperature.at(1)      = recvState.jointState[1].state[0].temperature;
    lowstate->errorstate.at(1)       = recvState.jointState[1].state[0].error;
    lowstate->isMotorConnected.at(1) = recvState.jointState[1].state[0].isConnected.state;
    //motor 2
    lowstate->temperature.at(2)      = recvState.jointState[1].state[1].temperature;
    lowstate->errorstate.at(2)       = recvState.jointState[1].state[1].error;
    lowstate->isMotorConnected.at(2) = recvState.jointState[1].state[1].isConnected.state;
    //motor 3-7
    for(int i(2); i<7; i++){
        lowstate->temperature.at(i+1)   = recvState.jointState[i].state[0].temperature;
        lowstate->errorstate.at(i+1)    = recvState.jointState[i].state[0].error;
        lowstate->isMotorConnected.at(i+1) = recvState.jointState[i].state[0].isConnected.state;
    }
    for(int i(0); i<7; i++){
        lowstate->errorstate.at(i) = lowstate->errorstate.at(i) & 0xBF;// 0x40: nothing
    }
}

void CtrlComponents::armCtrl(Vec6 q, Vec6 qd, Vec6 tau){
    lowcmd->setQ(q);
    lowcmd->setQd(qd);
    lowcmd->setTau(tau);
}

void CtrlComponents::gripperCtrl(double gripperPos, double gripperW, double gripperTau){
    lowcmd->setGripperQ(gripperPos);
    lowcmd->setGripperQd(gripperW);
    lowcmd->setGripperTau(gripperTau);
}