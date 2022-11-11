#include "control/unitreeArm.h"

unitreeArm::unitreeArm(CtrlComponents * ctrlComp):_ctrlComp(ctrlComp){
    sendRecvThread = new LoopFunc("Z1Communication", _ctrlComp->dt, boost::bind(&unitreeArm::sendRecv, this));
}

unitreeArm::~unitreeArm() {
    delete sendRecvThread;
}

void unitreeArm::setFsm(ArmFSMState fsm){
    _ctrlComp->sendCmd.state = fsm;
    if(fsm == ArmFSMState::LOWCMD){
        if(_ctrlComp->recvState.state != ArmFSMState::PASSIVE){
            _ctrlComp->sendCmd.state = ArmFSMState::PASSIVE;
            std::cout << "[ERROR] Only state_passive could tranfer to state_lowcmd" << std::endl;
        }else{
            _ctrlComp->lowcmd->setControlGain();
            _ctrlComp->lowcmd->setGripperGain();
        }
    }else{
        while (_ctrlComp->recvState.state != fsm){
            usleep(_ctrlComp->dt * 1000000);
        }
    }
}

void unitreeArm::backToStart(){
    setFsm(ArmFSMState::BACKTOSTART);
    while (_ctrlComp->recvState.state != ArmFSMState::JOINTCTRL){
        usleep(_ctrlComp->dt * 1000000);
    }
}

// move the arm to the posture indicated by the label
// the specific label name can be viewed in savedArmStates.csv
void unitreeArm::labelRun(std::string label){
    strcpy(_ctrlComp->sendCmd.valueUnion.name, label.c_str());
    setFsm(ArmFSMState::TOSTATE);
    while (_ctrlComp->recvState.state != ArmFSMState::JOINTCTRL){
        usleep(_ctrlComp->dt * 1000000);
    }
}

// save current posture to a label
// the label name should be less than 10 chars
void unitreeArm::labelSave(std::string label){
    strcpy(_ctrlComp->sendCmd.valueUnion.name, label.c_str());
    setFsm(ArmFSMState::SAVESTATE);
    while (_ctrlComp->recvState.state != ArmFSMState::JOINTCTRL){
        usleep(_ctrlComp->dt * 1000000);
    }
}

void unitreeArm::teach(std::string label){
    strcpy(_ctrlComp->sendCmd.valueUnion.name, label.c_str());
    setFsm(ArmFSMState::TEACH);
}

void unitreeArm::teachRepeat(std::string label){
    strcpy(_ctrlComp->sendCmd.valueUnion.name, label.c_str());
    setFsm(ArmFSMState::TEACHREPEAT);
    while (_ctrlComp->recvState.state != ArmFSMState::JOINTCTRL){
        usleep(_ctrlComp->dt * 1000000);
    }
}

void unitreeArm::calibration(){
    setFsm(ArmFSMState::CALIBRATION);
    while (_ctrlComp->recvState.state != ArmFSMState::PASSIVE){
        usleep(_ctrlComp->dt * 1000000);
    }
}

// reach the target posture by directly moving the joint
void unitreeArm::MoveJ(Vec6 posture, double maxSpeed) {
    _ctrlComp->sendCmd.valueUnion.trajCmd.posture[0] = Vec6toPosture(posture);
    _ctrlComp->sendCmd.valueUnion.trajCmd.maxSpeed = maxSpeed;
    setFsm(ArmFSMState::MOVEJ);
    while (_ctrlComp->recvState.state != ArmFSMState::JOINTCTRL){
        usleep(_ctrlComp->dt * 1000000);
    }
}

void unitreeArm::MoveJ(Vec6 posture, double gripperPos, double maxSpeed) {
    _ctrlComp->sendCmd.valueUnion.trajCmd.gripperPos = gripperPos;
    MoveJ(posture, maxSpeed);
}

// the end effector reaches the target point in a straight line trajectory
void unitreeArm::MoveL(Vec6 posture, double maxSpeed) {
    _ctrlComp->sendCmd.valueUnion.trajCmd.posture[0] = Vec6toPosture(posture);
    _ctrlComp->sendCmd.valueUnion.trajCmd.maxSpeed = maxSpeed;
    setFsm(ArmFSMState::MOVEL);
    while (_ctrlComp->recvState.state != ArmFSMState::JOINTCTRL){
        usleep(_ctrlComp->dt * 1000000);
    }
}

void unitreeArm::MoveL(Vec6 posture, double gripperPos, double maxSpeed) {
    _ctrlComp->sendCmd.valueUnion.trajCmd.gripperPos = gripperPos;
    MoveL(posture, maxSpeed);
}

// the end effector reaches the target point in a circular arc trajectory
void unitreeArm::MoveC(Vec6 middlePosutre, Vec6 endPosture, double maxSpeed) {
    _ctrlComp->sendCmd.valueUnion.trajCmd.posture[0] = Vec6toPosture(middlePosutre);
    _ctrlComp->sendCmd.valueUnion.trajCmd.posture[1] = Vec6toPosture(endPosture);
    _ctrlComp->sendCmd.valueUnion.trajCmd.maxSpeed = maxSpeed;
    setFsm(ArmFSMState::MOVEC);
    while (_ctrlComp->recvState.state != ArmFSMState::JOINTCTRL){
        usleep(_ctrlComp->dt * 1000000);
    }
}

void unitreeArm::MoveC(Vec6 middlePosutre, Vec6 endPosture, double gripperPos, double maxSpeed) {
    _ctrlComp->sendCmd.valueUnion.trajCmd.gripperPos = gripperPos;
    MoveC(middlePosutre, endPosture, maxSpeed);
}

void unitreeArm::setTraj(){
    if(_ctrlComp->recvState.state != ArmFSMState::SETTRAJ){
        _ctrlComp->sendCmd.valueUnion.trajCmd.trajOrder = 0;
        setFsm(ArmFSMState::SETTRAJ);
    }
    if(_ctrlComp->sendCmd.valueUnion.trajCmd.trajOrder == (_trajCmd.trajOrder - 1)){// make sure [order] is sequential
        _ctrlComp->sendCmd.valueUnion.trajCmd = _trajCmd; 
        usleep(10000);
    }
}

void unitreeArm::startTraj(){
    setFsm(ArmFSMState::JOINTCTRL);
    setFsm(ArmFSMState::TRAJECTORY);
}

void unitreeArm::startTrack(ArmFSMState fsm){
    switch(fsm){
        case ArmFSMState::JOINTCTRL:
                setFsm(ArmFSMState::JOINTCTRL);
                usleep(10000);//wait for statePast changed , in case of setting false automatically
                _ctrlComp->sendCmd.track = true;
                q  = _ctrlComp->lowstate->getQ();
                gripperQ = _ctrlComp->lowstate->getGripperQ();
                qd.setZero();
                gripperW = 0.;
            break;
        case ArmFSMState::CARTESIAN:
                setFsm(ArmFSMState::CARTESIAN);
                usleep(10000);
                _ctrlComp->sendCmd.track = true;
                // the current posture cmd must update to be consistent with the state
                _ctrlComp->lowcmd->endPosture = _ctrlComp->lowstate->endPosture;
                gripperQ = _ctrlComp->lowstate->getGripperQ();
            break;
        default:
                std::cout << "[ERROR] Please enter the state JOINTCTRL or CARTESION"<< std::endl;
            break;
    }
}

void unitreeArm::sendRecv(){
    switch (_ctrlComp->sendCmd.state)
    {
    case ArmFSMState::SAVESTATE:
    case ArmFSMState::TOSTATE:
    case ArmFSMState::TEACHREPEAT:
    case ArmFSMState::MOVEJ:
    case ArmFSMState::MOVEL:
    case ArmFSMState::MOVEC:
    case ArmFSMState::CALIBRATION:
    case ArmFSMState::TRAJECTORY:
    case ArmFSMState::BACKTOSTART:
    case ArmFSMState::TEACH:
    case ArmFSMState::PASSIVE:
        _ctrlComp->sendRecv();
        if(_ctrlComp->recvState.state == _ctrlComp->sendCmd.state){
            _ctrlComp->sendCmd.state = ArmFSMState::INVALID;
        }
        break;
    case ArmFSMState::JOINTCTRL:
        if(_ctrlComp->statePast != ArmFSMState::JOINTCTRL){
            _ctrlComp->sendCmd.track = false;
        }
        _ctrlComp->armCtrl(q, qd, Vec6::Zero());
        _ctrlComp->gripperCtrl(gripperQ, gripperW, 0.);
        _ctrlComp->sendRecv();
        break;
    case ArmFSMState::CARTESIAN:
        if(_ctrlComp->statePast != ArmFSMState::CARTESIAN){
            _ctrlComp->sendCmd.track = false;
        }
        _ctrlComp->gripperCtrl(gripperQ, 0., 0.);
        _ctrlComp->sendRecv();
        break;
    case ArmFSMState::SETTRAJ:
        _ctrlComp->sendCmd.valueUnion.trajCmd = _trajCmd;
        _ctrlComp->sendRecv();
        break;
    case ArmFSMState::LOWCMD:
        if(_ctrlComp->statePast != ArmFSMState::LOWCMD){
            q = _ctrlComp->lowstate->getQ();
            qd.setZero();
            tau.setZero();
            gripperQ = _ctrlComp->lowstate->getGripperQ();
            gripperW = 0.0;
            gripperTau = 0.0;
        }
        _ctrlComp->armCtrl(q, qd, tau);
        _ctrlComp->gripperCtrl(gripperQ, gripperW, gripperTau);
        _ctrlComp->sendRecv();
        break;
    case ArmFSMState::INVALID:
        _ctrlComp->sendRecv();
        break;
    default:
        break;
    }
}