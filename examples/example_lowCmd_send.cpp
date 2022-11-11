#include "control/unitreeArm.h"

class Z1ARM : public unitreeArm{
public:
    Z1ARM(CtrlComponents * ctrlComp):unitreeArm(ctrlComp){
        runThread = new LoopFunc("Z1LowCmd", _ctrlComp->dt, boost::bind(&Z1ARM::run, this));
    };
    ~Z1ARM(){delete runThread;};
    void run();
    LoopFunc *runThread;

    double direction;
    double velocity = 0.5;
};

void Z1ARM::run(){
    tau(0) = direction*3.;// torque
    q(1) += direction*_ctrlComp->dt*velocity;// hybrid, q & qd
    qd(1) = direction*velocity;
    q(2) -= direction*_ctrlComp->dt*velocity;
    qd(2) = direction*velocity;
    q(4) += direction*_ctrlComp->dt*velocity;// position
    qd(5) = direction*1.5;// velocity
    // gripper, if arm doesn't has gripper, it does noting.
    gripperQ -= direction*_ctrlComp->dt*velocity;
    sendRecv();
}

int main(int argc, char *argv[]) {
    CtrlComponents *ctrlComp = new CtrlComponents(0.002);
    Z1ARM arm(ctrlComp);
    arm.sendRecvThread->start();

    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.setFsm(ArmFSMState::LOWCMD);

    std::vector<double> KP, KW;
    KP = arm._ctrlComp->lowcmd->kp;
    KW = arm._ctrlComp->lowcmd->kd;

    // torque, only T
    KP.at(0) = 0.0;
    KW.at(0) = 0.0;
    // position, only kp
    KW.at(4) = 0.0;
    // velocity, only kd
    KP.at(5) = 0.0;
    arm._ctrlComp->lowcmd->setControlGain(KP, KW);
    arm.sendRecvThread->shutdown();
    arm.runThread->start();// using runThread instead of sendRecvThread

    for(int i(0); i<1000; i++){
        arm.direction = i < 600 ? 1. : -1.;
        usleep(arm._ctrlComp->dt*1000000);
    }

    arm.runThread->shutdown();
    arm.sendRecvThread->start();

    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    delete ctrlComp;
    return 0;
}