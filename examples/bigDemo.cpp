#include "unitreeArm.h"

class Custom : public unitreeArm{

public:
    Custom(){};
    ~Custom(){};
    void armCtrlByFSM();
    void armCtrlByTraj();
};

 
void Custom::armCtrlByFSM() {
    Vec6 posture[2];

    std::cout << "[JOINTCTRL]" << std::endl;
    setFsm(ArmFSMState::JOINTCTRL);
    sleep(1);// wait for a while for the command to execute

    std::cout << "[TOSTATE] forward" << std::endl;
    labelRun("forward");
    sleep(1);

    std::cout << "[MOVEJ]" << std::endl;
    posture[0]<<0.5,0.1,0.1,0.5,-0.2,0.5;
    MoveJ(posture[0], -1.0);
    sleep(1);

    std::cout << "[MOVEL]" << std::endl;
    posture[0] << 0,0,0,0.45,-0.2,0.2;
    MoveL(posture[0]);
    sleep(1);
    
    std::cout << "[MOVEC]" << std::endl;
    posture[0] << 0,0,0,0.4,0,0.3;
    posture[1] << 0,0,0,0.45,0.2,0.2;
    MoveC(posture[0], posture[1]);
    sleep(1);

    std::cout << "[BACKTOSTART]" << std::endl;
    backToStart();
    sleep(1);
}

void Custom::armCtrlByTraj(){
    Vec6 posture[2];

    std::cout << "[TOSTATE] forward" << std::endl;
    labelRun("forward");
    sleep(1);

    int order=1;

    // No.1 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::MoveJ;
    _trajCmd.maxSpeed = 0.3;
    _trajCmd.gripperPos = -1.0;
    posture[0] << 0.5,0.1,0.1,0.5,-0.2,0.5;
    // posture[0] << 0.0,0.0,0,0.5,-0.2,0.5;
    _trajCmd.posture[0] = Vec6toPosture(posture[0]);
    setTraj(_trajCmd);
    usleep(500000);

    // No.2 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::MoveL;
    _trajCmd.maxSpeed = 0.2;
    _trajCmd.gripperPos = 0.0;
    posture[0] << 0,0,0,0.45,-0.2,0.2;
    _trajCmd.posture[0] = Vec6toPosture(posture[0]);
    setTraj(_trajCmd);
    usleep(500000);

    // No.4 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::Stop;
    _trajCmd.stopTime = 2.0;
    _trajCmd.gripperPos = 0.0;
    setTraj(_trajCmd);
    usleep(500000);

    // No.4 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::MoveC;
    _trajCmd.maxSpeed = 0.3;
    _trajCmd.gripperPos = -1.0;
    posture[0] << 0,0,0,0.45,0,0.4;
    posture[1] << 0,0,0,0.45,0.2,0.2;
    _trajCmd.posture[0] = Vec6toPosture(posture[0]);
    _trajCmd.posture[1] = Vec6toPosture(posture[1]);
    setTraj(_trajCmd);
    usleep(500000);

    //run
    setFsm(ArmFSMState::TRAJECTORY);

    // wait for completion
    while (_recvState.state != ArmFSMState::JOINTCTRL){
        usleep(4000);
    }
}


int main() {
    Custom custom;

    custom.backToStart();
    sleep(1);

    if(false){
        custom.armCtrlByFSM();
    }else{
        custom.armCtrlByTraj();
    }

    custom.backToStart();
    return 0;
}