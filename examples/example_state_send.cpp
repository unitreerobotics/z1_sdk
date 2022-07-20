/*****************************************************************
Copyright (c) 2022, Unitree Robotics.Co.Ltd. All rights reserved.
*****************************************************************/
#include "unitree_arm_sdk/unitree_arm_sdk.h"


SendCmd sendCmd;
RecvState recvState;
UDPPort udp("127.0.0.1", 8071, 8072, sizeof(RecvState), BlockYN::NO, 500000);

void UDPCommunication(){
    udp.send((uint8_t*)&sendCmd, sizeof(SendCmd));
    udp.recv((uint8_t*)&recvState, sizeof(RecvState));
}

int main(){
    sendCmd = {0};
    recvState = {0};
    sendCmd.head[0] = 0xFE;
    sendCmd.head[1] = 0xFF;
    sendCmd.state = ArmFSMState::INVALID;

    float deltaTime = 4000;    // us

    LoopFunc *udpCommunication;
    udpCommunication = new LoopFunc("udp", 0.002, boost::bind(&UDPCommunication));
    udpCommunication->start();

std::cout << "[PASSIVE]" << std::endl;
    sendCmd.state = ArmFSMState::PASSIVE;
    while (recvState.state != ArmFSMState::PASSIVE){
        usleep(deltaTime);
    }

std::cout << "[BACKTOSTART]" << std::endl;
    sendCmd.state = ArmFSMState::BACKTOSTART;
    while (recvState.state != ArmFSMState::BACKTOSTART){
        usleep(deltaTime);
    }
    sendCmd.state = ArmFSMState::INVALID;
    while (recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }

std::cout << "[CARTESIAN]" << std::endl;
    sendCmd.state = ArmFSMState::CARTESIAN;
    while (recvState.state != ArmFSMState::CARTESIAN){
        usleep(deltaTime);
    }

    for(int i(0); i<2000; ++i){
        if(i<500){
            sendCmd.state = ArmFSMState::INVALID;
            sendCmd.value = ArmFSMValue::E;
        }
        else if(i<1000){
            sendCmd.state = ArmFSMState::INVALID;
            sendCmd.value = ArmFSMValue::Q;
        }
        else if(i<1500){
            sendCmd.state = ArmFSMState::INVALID;
            sendCmd.value = ArmFSMValue::D;
        }
        else{
            sendCmd.state = ArmFSMState::INVALID;
            sendCmd.value = ArmFSMValue::Q;
        }

        // printf("------state:%d-------\n", (int)recvState.state);
        // printf("Pos:%f \n", recvState.jointState[0].Pos);
        // printf("Pos:%f \n", recvState.jointState[1].Pos);
        // printf("Pos:%f \n", recvState.jointState[2].Pos);
        // printf("roll:%f \n", recvState.cartesianState.roll);
        // printf("pitch:%f \n", recvState.cartesianState.pitch);
        // printf("yaw:%f \n", recvState.cartesianState.yaw);
        usleep(deltaTime);
    }

std::cout << "[JOINTCTRL]" << std::endl;
    sendCmd.state = ArmFSMState::JOINTCTRL;
    while (recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }

    for(int i(0); i<1800; ++i){
        if(i<300){
            sendCmd.state = ArmFSMState::INVALID;
            sendCmd.value = ArmFSMValue::Q;
        }
        else if(i<600){
            sendCmd.state = ArmFSMState::INVALID;
            sendCmd.value = ArmFSMValue::W;
        }
        else if(i<900){
            sendCmd.state = ArmFSMState::INVALID;
            sendCmd.value = ArmFSMValue::D;
        }
        else if(i<1200){
            sendCmd.state = ArmFSMState::INVALID;
            sendCmd.value = ArmFSMValue::R;
        }
        else if(i<1500){
            sendCmd.state = ArmFSMState::INVALID;
            sendCmd.value = ArmFSMValue::T;
        }
        else{
            sendCmd.state = ArmFSMState::INVALID;
            sendCmd.value = ArmFSMValue::Y;
        }

        // printf("------state:%d-------\n", (int)recvState.state);
        // printf("Pos:%f \n", recvState.jointState[0].Pos);
        // printf("Pos:%f \n", recvState.jointState[1].Pos);
        // printf("Pos:%f \n", recvState.jointState[2].Pos);
        // printf("roll:%f \n", recvState.cartesianState.roll);
        // printf("pitch:%f \n", recvState.cartesianState.pitch);
        // printf("yaw:%f \n", recvState.cartesianState.yaw);
        usleep(deltaTime);
    }

std::cout << "[SAVESTATE]" << std::endl;
    std::string test = "test";
    sendCmd.state = ArmFSMState::SAVESTATE;
    strcpy(sendCmd.valueUnion.saveState, test.c_str());
    usleep(deltaTime);

std::cout << "[BACKTOSTART]" << std::endl;
    sendCmd.state = ArmFSMState::BACKTOSTART;
    while(recvState.state != ArmFSMState::BACKTOSTART){
        usleep(deltaTime);
    }
    sendCmd.state = ArmFSMState::INVALID;
    while (recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }


std::cout << "[TOSTATE]" << std::endl;
    sendCmd.state = ArmFSMState::TOSTATE;
    strcpy(sendCmd.valueUnion.toState, test.c_str());
    while (recvState.state != ArmFSMState::TOSTATE){
        usleep(deltaTime);
    }
    sendCmd.state = ArmFSMState::INVALID;
    while (recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }
    
std::cout << "[BACKTOSTART]" << std::endl;
    sendCmd.state = ArmFSMState::BACKTOSTART;
    while (recvState.state != ArmFSMState::BACKTOSTART){
        usleep(deltaTime);
    }
    sendCmd.state = ArmFSMState::INVALID;
    while (recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }

std::cout << "[MoveJ]" << std::endl;
    sendCmd.state = ArmFSMState::MOVEJ;
    sendCmd.valueUnion.moveJ.roll = 0;
    sendCmd.valueUnion.moveJ.pitch = 0;
    sendCmd.valueUnion.moveJ.yaw = 0;
    sendCmd.valueUnion.moveJ.x = 0.5;
    sendCmd.valueUnion.moveJ.y = -0.3;
    sendCmd.valueUnion.moveJ.z = 0.5;
    while (recvState.state != ArmFSMState::MOVEJ){
        usleep(deltaTime);
    }
    sendCmd.state = ArmFSMState::INVALID;
    while (recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }

std::cout << "[MoveL]" << std::endl;
    sendCmd.state = ArmFSMState::MOVEL;
    sendCmd.valueUnion.moveL.roll = 0;
    sendCmd.valueUnion.moveL.pitch = 0;
    sendCmd.valueUnion.moveL.yaw = 0;
    sendCmd.valueUnion.moveL.x = 0.5;
    sendCmd.valueUnion.moveL.y = 0.4;
    sendCmd.valueUnion.moveL.z = 0.3;
    while (recvState.state != ArmFSMState::MOVEL){
        usleep(deltaTime);
    }
    sendCmd.state = ArmFSMState::INVALID;
    while (recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }

std::cout << "[MoveC]" << std::endl;
    sendCmd.state = ArmFSMState::MOVEC;
    sendCmd.valueUnion.moveC[0].roll = 0;
    sendCmd.valueUnion.moveC[0].pitch = 0;
    sendCmd.valueUnion.moveC[0].yaw = 0;
    sendCmd.valueUnion.moveC[0].x = 0.5;
    sendCmd.valueUnion.moveC[0].y = 0;
    sendCmd.valueUnion.moveC[0].z = 0.5;
    sendCmd.valueUnion.moveC[1].roll = 0;
    sendCmd.valueUnion.moveC[1].pitch = 0;
    sendCmd.valueUnion.moveC[1].yaw = 0;
    sendCmd.valueUnion.moveC[1].x = 0.5;
    sendCmd.valueUnion.moveC[1].y = -0.4;
    sendCmd.valueUnion.moveC[1].z = 0.3;
    while (recvState.state != ArmFSMState::MOVEC){
        usleep(deltaTime);
    }
    sendCmd.state = ArmFSMState::INVALID;
    while (recvState.state != ArmFSMState::JOINTCTRL){
        usleep(deltaTime);
    }

std::cout << "[BACKTOSTART]" << std::endl;
    sendCmd.state = ArmFSMState::BACKTOSTART;
    while (recvState.state != ArmFSMState::BACKTOSTART){
        usleep(deltaTime);
    }
    return 0;
}