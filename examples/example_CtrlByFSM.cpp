#include "control/unitreeArm.h"

int main() {
    Vec6 posture[2];
    CtrlComponents *ctrlComp = new CtrlComponents(0.002);
    unitreeArm arm(ctrlComp);
    arm.sendRecvThread->start();

    arm.backToStart();
    
    //example
    std::cout << "[JOINTCTRL]" << std::endl;
    arm.setFsm(ArmFSMState::JOINTCTRL);

    std::cout << "[TO STATE]" << std::endl;
    arm.labelRun("forward");
    
    std::cout << "[MOVEJ]" << std::endl;
    posture[0]<<0.5,0.1,0.1,0.5,-0.2,0.5;
    arm.MoveJ(posture[0], -M_PI/3.0, 1.0);

    std::cout << "[MOVEL]" << std::endl;
    posture[0] << 0,0,0,0.45,-0.2,0.2;
    arm.MoveL(posture[0], 0., 0.3);
    
    std::cout << "[MOVEC]" << std::endl;
    posture[0] << 0,0,0,0.45,0,0.4;
    posture[1] << 0,0,0,0.45,0.2,0.2;
    arm.MoveC(posture[0], posture[1], -M_PI/3.0, 0.3);

    //stop
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    delete ctrlComp;
    return 0;
}