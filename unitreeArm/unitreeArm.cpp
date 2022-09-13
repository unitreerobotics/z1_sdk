#include "unitreeArm.h"

// constructor
// since the SDK communicates with z1_ctrl, the udp is set to the loopback address by default
unitreeArm::unitreeArm():_udp("127.0.0.1", 8071, 8072, sizeof(RecvState), BlockYN::NO, 500000) {
    _sendCmd.head[0] = 0xFE;
    _sendCmd.head[1] = 0xFF;

    _udpThread = new LoopFunc("udp", 0.004, boost::bind(&unitreeArm::UDPSendRecv, this));
    _udpThread->start();
}

unitreeArm::~unitreeArm() {
    delete _udpThread;
}

// udp send and receive function
// it has created a thread in the constructor that used to automatically process the arm state and command datas
void unitreeArm::UDPSendRecv() {
    _udp.send((uint8_t*)&_sendCmd, sizeof(SendCmd));
    _udp.recv((uint8_t*)&_recvState, sizeof(RecvState));
}

// set current FSM to fsm, this is equivalent to key switching when using keyboard control
// input: fsm     [the specific format can be viewed in class ArmFSMState ]
void unitreeArm::setFsm(ArmFSMState fsm) {
    while (_recvState.state != fsm){
        _sendCmd.state = fsm;
        usleep(deltaTime);
    }
}

void unitreeArm::backToStart() {
    setFsm(ArmFSMState::BACKTOSTART);
    //for those states that automatically transion to the joint space state, it's recommmanded to wait for it to finish
    _sendCmd.state = ArmFSMState::INVALID;
    while (_recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }
}

// move the arm to the posture indicated by the label
// the specific label name can be viewed in savedArmStates.csv
void unitreeArm::labelRun(std::string label) {
    while (_recvState.state != ArmFSMState::TOSTATE){
        _sendCmd.state = ArmFSMState::TOSTATE;
        strcpy(_sendCmd.valueUnion.name, label.c_str());
        usleep(deltaTime);
    }
    _sendCmd.state = ArmFSMState::INVALID;
    while (_recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }
}

// save current posture to a label
// the label name should be less than 10 chars
void unitreeArm::labelSave(std::string label) {
    _sendCmd.state = ArmFSMState::SAVESTATE;
    strcpy(_sendCmd.valueUnion.name, label.c_str());
    while (_recvState.state != ArmFSMState::SAVESTATE){
        usleep(deltaTime);
    }
}

// reach the target posture by directly moving the joint
void unitreeArm::MoveJ(Vec6 moveJCmd) {
    _sendCmd.valueUnion.trajCmd.posture[0] = Vec6toPosture(moveJCmd);
    
    _sendCmd.state = ArmFSMState::MOVEJ;
    while (_recvState.state != ArmFSMState::MOVEJ){
        usleep(deltaTime);
    }
    _sendCmd.state = ArmFSMState::INVALID;
    while (_recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }
}

// reach the target posture by directly moving the joint
void unitreeArm::MoveJ(Vec6 moveJCmd, double gripperPos) {
    _sendCmd.valueUnion.jointCmd[6].Pos = gripperPos;
    MoveJ(moveJCmd);
}

// the end effector reaches the target point in a straight line trajectory
void unitreeArm::MoveL(Vec6 moveLCmd) {
    _sendCmd.valueUnion.trajCmd.posture[0] = Vec6toPosture(moveLCmd);

    _sendCmd.state = ArmFSMState::MOVEL;
    while (_recvState.state != ArmFSMState::MOVEL){
        usleep(deltaTime);
    }
    _sendCmd.state = ArmFSMState::INVALID;
    while (_recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }
}

// the end effector reaches the target point in a circular arc trajectory
void unitreeArm::MoveC(Vec6 middleP, Vec6 endP){
    _sendCmd.valueUnion.trajCmd.posture[0] = Vec6toPosture(middleP);
    _sendCmd.valueUnion.trajCmd.posture[1] = Vec6toPosture(endP);

    _sendCmd.state = ArmFSMState::MOVEC;
    while (_recvState.state != ArmFSMState::MOVEC){
        usleep(deltaTime);
    }
    _sendCmd.state = ArmFSMState::INVALID;
    while (_recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }
}

void unitreeArm::getJointState(JointState* jointState) {
    for(int i=0;i<7;i++) {
        jointState[i] = _recvState.jointState[i];
    }
}

void unitreeArm::getGripperState(Posture& gripperState) {
    gripperState = _recvState.cartesianState;
}

void unitreeArm::setTraj(TrajCmd trajCmd){
    if(_recvState.state != ArmFSMState::SETTRAJ){
        _sendCmd.valueUnion.trajCmd.trajOrder = 0;
        setFsm(ArmFSMState::SETTRAJ);
    }

    if(_sendCmd.valueUnion.trajCmd.trajOrder == (trajCmd.trajOrder - 1)){// make sure [order] is sequential
        _sendCmd.valueUnion.trajCmd = trajCmd; 
    }
}
