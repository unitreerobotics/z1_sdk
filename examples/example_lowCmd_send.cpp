/*****************************************************************
Copyright (c) 2022, Unitree Robotics.Co.Ltd. All rights reserved.
*****************************************************************/
#include <math.h>
#include "unitree_arm_sdk/unitree_arm_sdk.h"

struct ConfigPD{
    float K_P[7];
    float K_W[7];

    virtual void setContrlGain()=0;
};

struct SimulationPD : ConfigPD{

    void setContrlGain(){
        K_P[0] = 80;
        K_P[1] = 200;
        K_P[2] = 200;
        K_P[3] = 15;
        K_P[4] = 15;
        K_P[5] = 15;
        K_P[6] = 15;
        
        K_W[0] = 5;
        K_W[1] = 10;
        K_W[2] = 10;
        K_W[3] = 2;
        K_W[4] = 2;
        K_W[5] = 2;
        K_W[6] = 5;
    }
};


struct RealRobotPD : ConfigPD{

    void setContrlGain(){
        K_P[0] = 20;
        K_P[1] = 40;
        K_P[2] = 20;
        K_P[3] = 160;
        K_P[4] = 160;
        K_P[5] = 160;
        K_P[6] = 20;

        K_W[0] = 6000;
        K_W[1] = 6000;
        K_W[2] = 6000;
        K_W[3] = 6000;
        K_W[4] = 6000;
        K_W[5] = 6000;
        K_W[6] = 3000;
    }
};

void printUsage() {
  printf(
      "Usage: controller [sim-or-robot]\n"
      "\t      sim-or-robot: s for sim, r for robot\n");
}

int main(int argc, char *argv []){
    // input number
    if (argc != 2) {
        printUsage();
        return EXIT_FAILURE;
    }

    // PD config
    ConfigPD *configPD;
    if (argv[1][0] == 's') {
        configPD = new SimulationPD();
        configPD->setContrlGain();
    } else if (argv[1][0] == 'r') {
        configPD = new RealRobotPD();
        configPD->setContrlGain();
    } else {
        printUsage();
        return EXIT_FAILURE;
    }

    // UDP object
    UDPPort udp("127.0.0.1", 8071, 8072, sizeof(RecvState), BlockYN::NO, 500000);
    
    // Send object
    SendCmd sendCmd;
    RecvState recvState;
    sendCmd = {0};
    recvState = {0};
    sendCmd.head[0] = 0xFE;
    sendCmd.head[1] = 0xFF;

    float deltaTime = 0.004;    // s

    // Only Passive state can change to LowCmd state
    while (recvState.state != ArmFSMState::PASSIVE){
        sendCmd.state = ArmFSMState::PASSIVE;
        udp.send((uint8_t*)&sendCmd, sizeof(SendCmd));
        udp.recv((uint8_t*)&recvState, sizeof(RecvState));
        usleep(deltaTime*1000000);
    }

    // LowCmd state
    sendCmd.state = ArmFSMState::LOWCMD;
    for(uint8_t i(0); i<7; ++i){
        sendCmd.valueUnion.jointCmd[i].Pos = 0;
        sendCmd.valueUnion.jointCmd[i].W = 0;
        sendCmd.valueUnion.jointCmd[i].T = 0;
        sendCmd.valueUnion.jointCmd[i].K_P = configPD->K_P[i];
        sendCmd.valueUnion.jointCmd[i].K_W = configPD->K_W[i];
    }
    sendCmd.valueUnion.jointCmd[3].Pos = -M_PI*4.26/180;
    sendCmd.valueUnion.jointCmd[3].W = 0;
    sendCmd.valueUnion.jointCmd[3].T = 0;
    sendCmd.valueUnion.jointCmd[3].K_P = configPD->K_P[3];
    sendCmd.valueUnion.jointCmd[3].K_W = configPD->K_W[3];

    int count = 0;
    float velocity = 1;         // rad/s

    for(int i(0); i<1000; ++i){
        if(i<500){
            // // torque
            // sendCmd.valueUnion.jointCmd[0].T = 2.5;
            // sendCmd.valueUnion.jointCmd[0].K_P = 0;
            // sendCmd.valueUnion.jointCmd[0].K_W = 0;

            // // postion
            // sendCmd.valueUnion.jointCmd[0].Pos += deltaTime*velocity;
            // sendCmd.valueUnion.jointCmd[0].K_P = configPD->K_P[0];
            // sendCmd.valueUnion.jointCmd[0].K_W = configPD->K_W[0];

            // // velocity
            // sendCmd.valueUnion.jointCmd[0].W = velocity;
            // sendCmd.valueUnion.jointCmd[0].K_P = 0;
            // sendCmd.valueUnion.jointCmd[0].K_W = configPD->K_W[0];

            // Grepper
            sendCmd.valueUnion.jointCmd[6].Pos -= deltaTime*velocity;
            sendCmd.valueUnion.jointCmd[6].W = -velocity;
            sendCmd.valueUnion.jointCmd[6].K_P = configPD->K_P[6];
            sendCmd.valueUnion.jointCmd[6].K_W = configPD->K_W[6];

        } else{
            // // torque
            // sendCmd.valueUnion.jointCmd[0].T = -2.5;
            // sendCmd.valueUnion.jointCmd[0].K_P = 0;
            // sendCmd.valueUnion.jointCmd[0].K_W = 0;

            // // postion
            // sendCmd.valueUnion.jointCmd[0].Pos -= deltaTime*velocity;
            // sendCmd.valueUnion.jointCmd[0].K_P = configPD->K_P[0];
            // sendCmd.valueUnion.jointCmd[0].K_W = configPD->K_W[0];

            // // velocity
            // sendCmd.valueUnion.jointCmd[0].W = -velocity;
            // sendCmd.valueUnion.jointCmd[0].K_P = 0;
            // sendCmd.valueUnion.jointCmd[0].K_W = configPD->K_W[0];

            // Grepper
            sendCmd.valueUnion.jointCmd[6].Pos += deltaTime*velocity;
            sendCmd.valueUnion.jointCmd[6].W = velocity;
            sendCmd.valueUnion.jointCmd[6].K_P = configPD->K_P[6];
            sendCmd.valueUnion.jointCmd[6].K_W = configPD->K_W[6];
        }

        udp.send((uint8_t*)&sendCmd, sizeof(SendCmd));
        udp.recv((uint8_t*)&recvState, sizeof(RecvState));

        // printf("Pos:%f \n", recvState.jointState[0].Pos);
        // printf("roll:%f \n", recvState.cartesianState.roll);
        // printf("pitch:%f \n", recvState.cartesianState.pitch);
        // printf("yaw:%f \n", recvState.cartesianState.yaw);
        usleep(deltaTime*1000000);
    }

    while (recvState.state != ArmFSMState::BACKTOSTART){
        sendCmd.state = ArmFSMState::BACKTOSTART;
        udp.send((uint8_t*)&sendCmd, sizeof(SendCmd));
        udp.recv((uint8_t*)&recvState, sizeof(RecvState));
        usleep(deltaTime*1000000);
    }

    delete configPD;
    return 0;
}