/*****************************************************************
Copyright (c) 2022, Unitree Robotics.Co.Ltd. All rights reserved.
*****************************************************************/
#include "unitree_arm_sdk/unitree_arm_sdk.h"

int main(){
    UDPPort udp("127.0.0.1", 8072, 8071, sizeof(SendCmd), BlockYN::NO, 500000);
    
    SendCmd sendCmd;
    RecvState recvState;
    sendCmd = {0};
    recvState = {0};

    recvState.head[0] = 0xFE;
    recvState.head[1] = 0xFF;

    bool turn = true;
    while(true){

        udp.send((uint8_t*)&recvState, sizeof(RecvState));
        udp.recv((uint8_t*)&sendCmd, sizeof(SendCmd));

        printf("%d \n", (int)sendCmd.state);
        usleep(2000);
    }
    return 0;
}