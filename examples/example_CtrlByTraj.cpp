#include "control/unitreeArm.h"

class Z1ARM : public unitreeArm{
public:
    Z1ARM(CtrlComponents * ctrlComp):unitreeArm(ctrlComp){};
    ~Z1ARM(){};
    void setJointTraj();
    void setLineTraj();
    void setCircleTraj();
    void armCtrlTrackInCart();
    void printState();
};

int main() {
    CtrlComponents *ctrlComp = new CtrlComponents(0.002);
    unitreeArm arm(ctrlComp);
    arm.sendRecvThread->start();

    arm.backToStart();

    //example
    Vec6 posture[2];
    int order = 1;

    arm.labelRun("forward");

    arm._trajCmd.trajOrder = 0;//if order == 0, clear traj
    arm.setTraj();

    // No.1 trajectory
    arm._trajCmd.trajOrder = order++;
    arm._trajCmd.trajType = TrajType::MoveJ;
    arm._trajCmd.maxSpeed = 1.0;// angular velocity, rad/s
    arm._trajCmd.gripperPos = 0.0;
    posture[0] << 0.5,0.1,0.1,0.5,-0.2,0.5;
    arm._trajCmd.posture[0] = Vec6toPosture(posture[0]);
    arm.setTraj();

    // No.2 trajectory
    arm._trajCmd.trajOrder = order++;
    arm._trajCmd.trajType = TrajType::Stop;
    arm._trajCmd.stopTime = 1.0;
    arm._trajCmd.gripperPos = -1.0;
    arm.setTraj();


    // No.3 trajectory
    arm._trajCmd.trajOrder = order++;
    arm._trajCmd.trajType = TrajType::MoveL;
    arm._trajCmd.maxSpeed = 0.3; // Cartesian velocity , m/s
    arm._trajCmd.gripperPos = 0.0;
    posture[0] << 0,0,0,0.45,-0.2,0.2;
    arm._trajCmd.posture[0] = Vec6toPosture(posture[0]);
    arm.setTraj();

    // No.4 trajectory
    arm._trajCmd.trajOrder = order++;
    arm._trajCmd.trajType = TrajType::Stop;
    arm._trajCmd.stopTime = 1.0; 
    arm._trajCmd.gripperPos = -1.0;
    arm.setTraj();

    // No.5 trajectory
    arm._trajCmd.trajOrder = order++;
    arm._trajCmd.trajType = TrajType::MoveC;
    arm._trajCmd.maxSpeed = 0.3; // Cartesian velocity
    arm._trajCmd.gripperPos = 0.0;
    posture[0] << 0,0,0,0.45,0,0.4;
    posture[1] << 0,0,0,0.45,0.2,0.2;
    arm._trajCmd.posture[0] = Vec6toPosture(posture[0]);
    arm._trajCmd.posture[1] = Vec6toPosture(posture[1]);
    arm.setTraj();

    arm.startTraj();
    // wait for trajectory completion
    while (arm._ctrlComp->recvState.state != ArmFSMState::JOINTCTRL){
        usleep(arm._ctrlComp->dt*1000000);
    }


    //stop
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    delete ctrlComp;
    return 0;
}