#include "control/unitreeArm.h"

using namespace UNITREE_ARM;

class Z1ARM : public unitreeArm{
public:
    Z1ARM():unitreeArm(true){};
    ~Z1ARM(){};
    void armCtrlByFSM();
    void armCtrlTrackInJointCtrl();
    void armCtrlTrackInCartesian();
    void printState();
private:
    Vec6 qPast;
};

 
void Z1ARM::armCtrlByFSM() {
    Vec6 posture[2];

    std::cout << "[JOINTCTRL]" << std::endl;
    setFsm(ArmFSMState::JOINTCTRL);

    std::cout << "[TO STATE]" << std::endl;
    labelRun("forward");
    
    std::cout << "[MOVEJ]" << std::endl;
    posture[0]<<0.5,0.1,0.1,0.5,-0.2,0.5;
    MoveJ(posture[0], -M_PI/3.0, 1.0);

    std::cout << "[MOVEL]" << std::endl;
    posture[0] << 0,0,0,0.45,-0.2,0.2;
    MoveL(posture[0], 0., 0.3);
    
    std::cout << "[MOVEC]" << std::endl;
    posture[0] << 0,0,0,0.45,0,0.4;
    posture[1] << 0,0,0,0.45,0.2,0.2;
    MoveC(posture[0], posture[1], -M_PI/3.0, 0.3);
}

void Z1ARM::armCtrlTrackInJointCtrl(){
    labelRun("forward");
    startTrack(ArmFSMState::JOINTCTRL);
    for(;;){
        q(3) -= _ctrlComp->dt * 1.0;//max dt*PI, rad/s

        qPast = lowstate->getQ();
        // std::cout << "qCmd: " << q.transpose() << " qState: " << qPast.transpose() << std::endl;
        //The joint has reached limit, there is warning: joint cmd is far from state
        double error = fabs(q(3) - qPast(3));
        if(error > 0.1){
            break;
        }
        usleep(_ctrlComp->dt*1000000);
    }
    _ctrlComp->sendCmd.track = false;
}

void Z1ARM::armCtrlTrackInCartesian(){
    labelRun("forward");
    startTrack(ArmFSMState::CARTESIAN);
    for(;;){
        endPosture(5) -= _ctrlComp->dt * 0.2;//z axis, m/s

        // no inverse kinematics solution, the joint has reached limit
        std::cout << "postureCmd: " << endPosture.transpose() << " qState: " << lowstate->endPosture.transpose() << std::endl;
        double error = fabs(endPosture(5) - lowstate->endPosture(5));
        if( error > 0.1){
            break;
        }
        usleep(_ctrlComp->dt*1000000);
    }
    _ctrlComp->sendCmd.track = false;
}

void Z1ARM::printState(){
    std::cout<<"------ joint State ------"<<std::endl;
    std::cout<<"qState: "<<lowstate->getQ().transpose()<<std::endl;
    std::cout<<"qdState: "<<lowstate->getQd().transpose()<<std::endl;
    std::cout<<"tauState: "<<lowstate->getTau().transpose()<<std::endl;

    std::cout<<"------ Endeffector Cartesian Posture ------"<<std::endl;
    std::cout<<"roll pitch yaw x y z"<<std::endl;
    std::cout<<lowstate->endPosture.transpose()<<std::endl;
}

int main() {
    std::cout << std::fixed << std::setprecision(3);
    Z1ARM arm;
    arm.sendRecvThread->start();

    arm.backToStart();

    // size_t demo = 3;
    for(size_t demo = 1; demo < 4; demo++)
    switch (demo)
    {
        case 1:
            arm.armCtrlByFSM();
            break;
        case 2:
            arm.armCtrlTrackInJointCtrl();
            break;
        case 3:
            arm.armCtrlTrackInCartesian();
            break;
        default:
            break;
    }
    
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    return 0;
}