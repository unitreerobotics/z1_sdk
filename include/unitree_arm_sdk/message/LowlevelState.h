#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include <vector>
#include "math/mathTypes.h"
#include "message/arm_common.h"

Vec6 PosturetoVec6(const Posture p);
Posture Vec6toPosture(const Vec6 p);

struct LowlevelState{
private:
    size_t _dof = 6;
public:
    LowlevelState();
    ~LowlevelState(){};

    Vec6 endPosture;
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> ddq;
    std::vector<double> tau;

    std::vector<int> temperature;
    std::vector<uint8_t> errorstate;
    std::vector<uint8_t> isMotorConnected;

    Vec6 getQ();
    Vec6 getQd();
    Vec6 getQdd();
    Vec6 getTau();
    double getGripperQ() {return q.at(q.size()-1);}
    double getGripperQd() {return dq.at(q.size()-1);}
    double getGripperTau() {return tau.at(tau.size()-1);}
};

#endif  //LOWLEVELSTATE_HPP
