#include "unitreeArm.h"

class Z1ARM : public unitreeArm{

public:
    Z1ARM(){};
    ~Z1ARM(){};
    void armCtrlByFSM();
    void armCtrlByTraj();
    void armCtrlTrackInJointCtrl();
    void armCtrlTrackInCartesian();
};

 
void Z1ARM::armCtrlByFSM() {
    Vec6 posture[2];

    std::cout << "[JOINTCTRL]" << std::endl;
    setFsm(ArmFSMState::JOINTCTRL);

    std::cout << "[TOSTATE] forward" << std::endl;
    labelRun("forward");

    std::cout << "[MOVEJ]" << std::endl;
    posture[0]<<0.5,0.1,0.1,0.5,-0.2,0.5;
    MoveJ(posture[0], -1.0);

    std::cout << "[MOVEL]" << std::endl;
    posture[0] << 0,0,0,0.45,-0.2,0.2;
    MoveL(posture[0]);
    
    std::cout << "[MOVEC]" << std::endl;
    posture[0] << 0,0,0,0.4,0,0.3;
    posture[1] << 0,0,0,0.45,0.2,0.2;
    MoveC(posture[0], posture[1]);
}

void Z1ARM::armCtrlByTraj(){
    Vec6 posture[2];
    int order=1;

    labelRun("forward");

    // No.1 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::MoveJ;
    _trajCmd.maxSpeed = 1.0;// angular velocity
    _trajCmd.gripperPos = 0.0;
    posture[0] << 0.5,0.1,0.1,0.5,-0.2,0.5;
    _trajCmd.posture[0] = Vec6toPosture(posture[0]);
    setTraj(_trajCmd);
    usleep(10000);

    // No.2 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::Stop;
    _trajCmd.stopTime = 1.0;
    _trajCmd.gripperPos = -1.0;
    setTraj(_trajCmd);
    usleep(10000);

    // No.3 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::MoveL;
    _trajCmd.maxSpeed = 0.3; // Cartesian velocity
    _trajCmd.gripperPos = 0.0;
    posture[0] << 0,0,0,0.45,-0.2,0.2;
    _trajCmd.posture[0] = Vec6toPosture(posture[0]);
    setTraj(_trajCmd);
    usleep(10000);

    // No.4 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::Stop;
    _trajCmd.stopTime = 1.0; 
    _trajCmd.gripperPos = -1.0;
    setTraj(_trajCmd);
    usleep(10000);

    // No.5 trajectory
    _trajCmd.trajOrder = order++;
    _trajCmd.trajType = TrajType::MoveC;
    _trajCmd.maxSpeed = 0.3; // Cartesian velocity
    _trajCmd.gripperPos = 0.0;
    posture[0] << 0,0,0,0.45,0,0.4;
    posture[1] << 0,0,0,0.45,0.2,0.2;
    _trajCmd.posture[0] = Vec6toPosture(posture[0]);
    _trajCmd.posture[1] = Vec6toPosture(posture[1]);
    setTraj(_trajCmd);
    usleep(10000);

    //run trajectory
    setFsm(ArmFSMState::TRAJECTORY);

    // wait for trajectory completion
    while (_recvState.state != ArmFSMState::JOINTCTRL){
        usleep(4000);
    }
}

void Z1ARM::armCtrlTrackInJointCtrl(){
    labelRun("forward");// auto change to JOINTCTRL state after TOSTATE
    for( size_t i(0); i < 6; i++ ){// the current joint cmd must update to be consistent with the state
        _sendCmd.valueUnion.jointCmd[i].Pos = _recvState.jointState[i].Pos;
        _sendCmd.valueUnion.jointCmd[i].W = 0.0;
    }
    // while track is true, the arm will track joint cmd (Only q and dq)
    _sendCmd.track = true;

    for(;;){
        _sendCmd.valueUnion.jointCmd[0].Pos += 0.002;

        //The joint has reached limit, there is warning: joint cmd is far from state
        double error = abs(_sendCmd.valueUnion.jointCmd[0].Pos - _recvState.jointState[0].Pos);
        if(error > 0.1){
            break;
        }
        usleep(4000);
    }
    _sendCmd.track = false;
}

void Z1ARM::armCtrlTrackInCartesian(){
    labelRun("forward");
    setFsm(ArmFSMState::CARTESIAN);
    // the current posture cmd must update to be consistent with the state
    _sendCmd.valueUnion.trajCmd.posture[0] = _recvState.cartesianState;
    // while track is true, the arm will track posture[0] cmd
    _sendCmd.track = true;
    for(;;){
        _sendCmd.valueUnion.trajCmd.posture[0].y += 0.0005;
        // std::cout << PosturetoVec6(_sendCmd.valueUnion.trajCmd.posture[0]).transpose() << std::endl;

        // no inverse kinematics solution, the joint has reached limit
        double error = (PosturetoVec6(_recvState.cartesianState) - PosturetoVec6(_sendCmd.valueUnion.trajCmd.posture[0])).norm();
        if( error > 0.1){
            break;
        }
        usleep(4000);
    }
    _sendCmd.track = false;
}

int main() {
    Z1ARM arm;

    arm.backToStart();
    sleep(1);

    size_t demo = 2;
    // for(size_t demo = 1; demo < 5; demo++)
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
    return 0;
}