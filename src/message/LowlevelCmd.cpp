#include "message/LowlevelCmd.h"

LowlevelCmd::LowlevelCmd(){
    q.resize(_dof+1);
    dq.resize(_dof+1);
    tau.resize(_dof+1);
    kp.resize(_dof+1);
    kd.resize(_dof+1);
}


void LowlevelCmd::setZeroDq(){
    for(size_t i(0); i<_dof; ++i){
        dq.at(i) = 0.0;
    }
}

void LowlevelCmd::setZeroTau(){
    for(size_t i(0); i<_dof; ++i){
        tau.at(i) = 0.0;
    }
}

void LowlevelCmd::setZeroKp(){
    for(size_t i(0); i<_dof; ++i){
        kp.at(i) = 0.0;
    }
}

void LowlevelCmd::setZeroKd(){
    for(size_t i(0); i<_dof; ++i){
        kd.at(i) = 0.0;
    }
}

void LowlevelCmd::setControlGain(std::vector<double> KP, std::vector<double> KW){
    kp.at(0) = KP.at(0);
    kp.at(1) = KP.at(1);
    kp.at(2) = KP.at(2);
    kp.at(3) = KP.at(3);
    kp.at(4) = KP.at(4);
    kp.at(5) = KP.at(5);

    kd.at(0) = KW.at(0);
    kd.at(1) = KW.at(1);
    kd.at(2) = KW.at(2);
    kd.at(3) = KW.at(3);
    kd.at(4) = KW.at(4);
    kd.at(5) = KW.at(5);
}

void LowlevelCmd::setControlGain(){
    kp.at(0) = 20;
    kp.at(1) = 30;
    kp.at(2) = 30;
    kp.at(3) = 20;
    kp.at(4) = 15;
    kp.at(5) = 10;

    kd.at(0) = 2000;
    kd.at(1) = 2000;
    kd.at(2) = 2000;
    kd.at(3) = 2000;
    kd.at(4) = 2000;
    kd.at(5) = 2000;
}

void LowlevelCmd::setQ(std::vector<double> qInput){
    if(qInput.size() != _dof){
        std::cout << "[ERROR] The qInput size of LowlevelCmd::setQ(std::vector<double>) is not suitable" << std::endl;
    }
    for(size_t i(0); i<_dof; ++i){
        q.at(i) = qInput.at(i);
    }
}

void LowlevelCmd::setQ(VecX qInput){
    if(qInput.rows() != _dof){
        std::cout << "[ERROR] The qInput size of LowlevelCmd::setQ(VecX) is not suitable" << std::endl;
    }
    for(size_t i(0); i<_dof; ++i){
        q.at(i) = qInput(i);
    }
}

void LowlevelCmd::setQd(VecX qDInput){
    if(qDInput.rows() != _dof){
        std::cout << "[ERROR] The qDInput size of LowlevelCmd::setQd(VecX) is not suitable" << std::endl;
    }
    for(size_t i(0); i<_dof; ++i){
        dq.at(i) = qDInput(i);
    }
}

void LowlevelCmd::setTau(VecX tauInput){
    if(tauInput.rows() != _dof){
        std::cout << "[ERROR] The tauInput size of LowlevelCmd::setTau(VecX) is not suitable" << std::endl;
    }
    for(size_t i(0); i<_dof; ++i){
        tau.at(i) = tauInput(i);
    }
}

void LowlevelCmd::setPassive(){
    setZeroDq();
    setZeroTau();
    setZeroKp();
    for(size_t i; i<_dof; ++i){
        kd.at(i) = 10.0;
    }
}

void LowlevelCmd::setGripperGain(double KP, double KW){
    kp.at(kp.size()-1) = KP;
    kd.at(kd.size()-1) = KW;
}

//set gripper default gain
void LowlevelCmd::setGripperGain(){
    kp.at(kp.size()-1) = 20.0;
    kd.at(kd.size()-1) = 100.0;
}

//set gripper gain to 0
void LowlevelCmd::setGripperZeroGain(){
    kp.at(kp.size()-1) = 0.0;
    kd.at(kd.size()-1) = 0.0;
}

void LowlevelCmd::setGripperQ(double qInput){
    q.at(q.size()-1) = qInput;
}

double LowlevelCmd::getGripperQ(){
    return q.at(q.size()-1);
}

void LowlevelCmd::setGripperQd(double qdInput){
    dq.at(dq.size()-1) = qdInput;
}

double LowlevelCmd::getGripperQd(){
    return dq.at(dq.size()-1);
}

void LowlevelCmd::setGripperTau(double tauInput){
    tau.at(tau.size()-1) = tauInput;
}

double LowlevelCmd::getGripperTau(){
    return tau.at(tau.size()-1);
}

//q of joints, without gripper
Vec6 LowlevelCmd::getQ(){
    Vec6 qReturn;
    for(int i(0); i<6; ++i){
        qReturn(i) = q.at(i);
    }
    return qReturn;
}

//qd of joints, without gripper
Vec6 LowlevelCmd::getQd(){
    Vec6 qdReturn;
    for(int i(0); i<6; ++i){
        qdReturn(i) = dq.at(i);
    }
    return qdReturn;
}
