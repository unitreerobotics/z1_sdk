#include "message/LowlevelState.h"

LowlevelState::LowlevelState(){
//set q, dq, tau parameters
    q.resize(_dof+1);
    dq.resize(_dof+1);
    ddq.resize(_dof+1);
    tau.resize(_dof+1);

    temperature.resize(_dof+2);
    errorstate.resize(_dof+2);
    isMotorConnected.resize(_dof+2);
}

Vec6 LowlevelState::getQ(){
    Vec6 qReturn;
    for(size_t i(0); i<_dof; ++i){
        qReturn(i) = q.at(i);
    }
    return qReturn;
}

Vec6 LowlevelState::getQd(){
    Vec6 qdReturn;
    for(size_t i(0); i<_dof; ++i){
        qdReturn(i) = dq.at(i);
    }
    return qdReturn;
}

Vec6 LowlevelState::getQdd(){
    Vec6 qddReturn;
    for(size_t i(0); i<_dof; ++i){
        qddReturn(i) = ddq.at(i);
    }
    return qddReturn;
}

Vec6 LowlevelState::getTau(){
    Vec6 tauReturn;
    for(int i(0); i < _dof; ++i){
        tauReturn(i) = tau.at(i);
    }
    return tauReturn;
}

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