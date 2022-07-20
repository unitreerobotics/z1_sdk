#ifndef _UNITREE_ARM_UDP_H_
#define _UNITREE_ARM_UDP_H_

#include <stdint.h>
#include <unistd.h>
#include <vector>
#include <arpa/inet.h>
#include <string>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>

#include "unitree_arm_sdk/common/arm_motor_common.h"

enum class BlockYN{
    YES,
    NO
};

class IOPort{
public:
    IOPort(BlockYN blockYN, size_t recvLength, size_t timeOutUs){
        resetIO(blockYN, recvLength, timeOutUs);
    }
    virtual ~IOPort(){}
    virtual size_t send(uint8_t *sendMsg, size_t sendLength) = 0;
    virtual size_t recv(uint8_t *recvMsg, size_t recvLength) = 0;
    virtual size_t recv(uint8_t *recvMsg) = 0;
    virtual bool sendRecv(std::vector<MOTOR_send> &sendVec, std::vector<MOTOR_recv> &recvVec) = 0;
    void resetIO(BlockYN blockYN, size_t recvLength, size_t timeOutUs);
protected:
    BlockYN _blockYN = BlockYN::NO;
    size_t _recvLength;
    timeval _timeout;
    timeval _timeoutSaved;
};


class UDPPort : public IOPort{
public:
    UDPPort(std::string toIP, uint toPort, uint ownPort, 
            size_t recvLength = 0,
            BlockYN blockYN = BlockYN::NO,
            size_t timeOutUs = 20000);
    ~UDPPort();
    size_t send(uint8_t *sendMsg, size_t sendMsgLength);
    size_t recv(uint8_t *recvMsg, size_t recvLength);
    size_t recv(uint8_t *recvMsg);
    bool sendRecv(std::vector<MOTOR_send> &sendVec, std::vector<MOTOR_recv> &recvVec);
private:
    size_t _recvBlock(uint8_t *recvMsg, size_t recvLength);
    size_t _recvUnBlock(uint8_t *recvMsg, size_t recvLength);
    sockaddr_in _ownAddr, _toAddr, _fromAddr;
    socklen_t _sockaddrSize;
    int _sockfd;
    int _on = 1;
    size_t _sentLength;

    uint8_t _sendBytes[238];    // 7 motors
    uint8_t _recvBytes[546];   // 7 motors

    fd_set _rSet;
};

#endif  // _UNITREE_ARM_UDP_H_