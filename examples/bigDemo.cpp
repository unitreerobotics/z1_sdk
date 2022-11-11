#include "control/unitreeArm.h"

class Z1ARM : public unitreeArm{
public:
    Z1ARM(CtrlComponents * ctrlComp):unitreeArm(ctrlComp){};
    ~Z1ARM(){};
    void armCtrlByFSM();
    void armCtrlByTraj();
    void armCtrlTrackInJointCtrl();
    void armCtrlTrackInCartesian();
    void printState();
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

void Z1ARM::armCtrlByTraj(){
    Vec6 posture[2];
    int order = 1;

    labelRun("forward");

    _trajCmd.trajOrder = 0;//if order == 0, clear traj
    setTraj();

    // No.1 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::MoveJ;
    _trajCmd.maxSpeed = 1.0;// angular velocity, rad/s
    _trajCmd.gripperPos = 0.0;
    posture[0] << 0.5,0.1,0.1,0.5,-0.2,0.5;
    _trajCmd.posture[0] = Vec6toPosture(posture[0]);
    setTraj();

    // No.2 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::Stop;
    _trajCmd.stopTime = 1.0;
    _trajCmd.gripperPos = -1.0;
    setTraj();


    // No.3 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::MoveL;
    _trajCmd.maxSpeed = 0.3; // Cartesian velocity , m/s
    _trajCmd.gripperPos = 0.0;
    posture[0] << 0,0,0,0.45,-0.2,0.2;
    _trajCmd.posture[0] = Vec6toPosture(posture[0]);
    setTraj();

    // No.4 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::Stop;
    _trajCmd.stopTime = 1.0; 
    _trajCmd.gripperPos = -1.0;
    setTraj();

    // No.5 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::MoveC;
    _trajCmd.maxSpeed = 0.3; // Cartesian velocity
    _trajCmd.gripperPos = 0.0;
    posture[0] << 0,0,0,0.45,0,0.4;
    posture[1] << 0,0,0,0.45,0.2,0.2;
    _trajCmd.posture[0] = Vec6toPosture(posture[0]);
    _trajCmd.posture[1] = Vec6toPosture(posture[1]);
    setTraj();

    startTraj();
    // wait for trajectory completion
    while (_ctrlComp->recvState.state != ArmFSMState::JOINTCTRL){
        usleep(_ctrlComp->dt*1000000);
    }
}

void Z1ARM::armCtrlTrackInJointCtrl(){
    labelRun("forward");
    startTrack(ArmFSMState::JOINTCTRL);
    for(;;){
        q(3) -= _ctrlComp->dt * 1.0;//max dt*PI, rad/s

        qPast = _ctrlComp->lowstate->getQ();
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
        _ctrlComp->lowcmd->endPosture(5) -= _ctrlComp->dt * 0.2;//z, m/s

        // no inverse kinematics solution, the joint has reached limit
        // std::cout << "postureCmd: " << _ctrlComp->lowcmd->endPosture.transpose() << " qState: " << _ctrlComp->lowstate->endPosture.transpose() << std::endl;
        double error = fabs(_ctrlComp->lowcmd->endPosture(5) - _ctrlComp->lowstate->endPosture(5));
        if( error > 0.1){
            break;
        }
        usleep(_ctrlComp->dt*1000000);
    }
    _ctrlComp->sendCmd.track = false;
}

void Z1ARM::printState(){
    std::cout<<"------ joint State ------"<<std::endl;
    std::cout<<"qState: "<<_ctrlComp->lowstate->getQ().transpose()<<std::endl;
    std::cout<<"qdState: "<<_ctrlComp->lowstate->getQd().transpose()<<std::endl;
    std::cout<<"tauState: "<<_ctrlComp->lowstate->getTau().transpose()<<std::endl;

    std::cout<<"------ Endeffector Cartesian Posture ------"<<std::endl;
    std::cout<<"roll pitch yaw x y z"<<std::endl;
    std::cout<<_ctrlComp->lowstate->endPosture.transpose()<<std::endl;
}

int main() {
    CtrlComponents *ctrlComp = new CtrlComponents(0.002);
    Z1ARM arm(ctrlComp);
    arm.sendRecvThread->start();

    arm.backToStart();

    // size_t demo = 2;
    for(size_t demo = 1; demo < 5; demo++)
    // for(;;)
    switch (demo)
    {
        case 1:
            arm.armCtrlByFSM();
            break;
        case 2:
            arm.armCtrlByTraj();
            break;
        case 3:
            arm.armCtrlTrackInJointCtrl();
            break;
        case 4:
            arm.armCtrlTrackInCartesian();
            break;
        default:
            break;
    }
    
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    delete ctrlComp;
    return 0;
}