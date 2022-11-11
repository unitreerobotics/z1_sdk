#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "math/mathTypes.h"
#include <vector>
#include <iostream>

struct LowlevelCmd{
private:
    size_t _dof = 6;
public:
    Vec6 endPosture;
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> tau;
    std::vector<double> kp;
    std::vector<double> kd;
    Vec6 posture;

    LowlevelCmd();
    ~LowlevelCmd(){}

    void setZeroDq();
    void setZeroTau();
    void setZeroKp();
    void setZeroKd();
    void setControlGain();
    void setControlGain(std::vector<double> KP, std::vector<double> KW);
    void setQ(std::vector<double> qInput);
    void setQ(VecX qInput);
    void setQd(VecX qDInput);
    void setTau(VecX tauInput);
    void setPassive();
    void setGripperGain();
    void setGripperGain(double KP, double KW);
    void setGripperZeroGain();
    void setGripperQ(double qInput);
    double getGripperQ();
    void setGripperQd(double qdInput);
    double getGripperQd();
    void setGripperTau(double tauInput);
    double getGripperTau();
    Vec6 getQ();
    Vec6 getQd();
};


#endif  //LOWLEVELCMD_H
