/*****************************************************************
Copyright (c) 2022, Unitree Robotics.Co.Ltd. All rights reserved.
*****************************************************************/
#include "unitree_arm_sdk/unitree_arm_sdk.h"

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig){
	std::cout << "[STATE] stop the controller" << std::endl;
    running = false;
}

int main(){
    EmptyAction emptyAction((int)ArmFSMState::INVALID);
    std::vector<KeyAction*> events;
    UnitreeKeyboardUDPSend *cmdPanel;

    events.push_back(new StateAction("`", (int)ArmFSMState::BACKTOSTART));
    events.push_back(new StateAction("1", (int)ArmFSMState::PASSIVE));
    events.push_back(new StateAction("2", (int)ArmFSMState::JOINTCTRL));
    events.push_back(new StateAction("3", (int)ArmFSMState::CARTESIAN));
    events.push_back(new StateAction("4", (int)ArmFSMState::MOVEJ));
    events.push_back(new StateAction("5", (int)ArmFSMState::MOVEL));
    events.push_back(new StateAction("6", (int)ArmFSMState::MOVEC));
    events.push_back(new StateAction("7", (int)ArmFSMState::TEACH));
    events.push_back(new StateAction("8", (int)ArmFSMState::TEACHREPEAT));
    events.push_back(new StateAction("9", (int)ArmFSMState::SAVESTATE));
    events.push_back(new StateAction("0", (int)ArmFSMState::TOSTATE));
    events.push_back(new StateAction("-", (int)ArmFSMState::TRAJECTORY));
    events.push_back(new StateAction("=", (int)ArmFSMState::CALIBRATION));
    events.push_back(new StateAction("]", (int)ArmFSMState::NEXT));
    events.push_back(new StateAction("/", (int)ArmFSMState::LOWCMD));

    events.push_back(new ValueAction("q", "a", 1.0));
    events.push_back(new ValueAction("w", "s", 1.0));
    events.push_back(new ValueAction("e", "d", 1.0));
    events.push_back(new ValueAction("r", "f", 1.0));
    events.push_back(new ValueAction("t", "g", 1.0));
    events.push_back(new ValueAction("y", "h", 1.0));
    events.push_back(new ValueAction("down", "up", 1.0));

    cmdPanel = new UnitreeKeyboardUDPSend(events, emptyAction);

    while(running){
        usleep(1000000);
    }

    delete cmdPanel;

    return 0;
}