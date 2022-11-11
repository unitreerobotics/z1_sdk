#include "message/udp.h"

void IOPort::resetIO(BlockYN blockYN, size_t recvLength, size_t timeOutUs){
    _blockYN = blockYN;
    _recvLength = recvLength;
    _timeout.tv_sec  = timeOutUs / 1000000;
    _timeout.tv_usec = timeOutUs % 1000000;
    _timeoutSaved = _timeout;
}

UDPPort::UDPPort(std::string toIP, uint toPort, uint ownPort, 
                        size_t recvLength,
                        BlockYN blockYN, size_t timeOutUs)
    :IOPort(blockYN, recvLength, timeOutUs){
    bzero(&_toAddr, sizeof(sockaddr_in));
    bzero(&_ownAddr, sizeof(sockaddr_in));
    bzero(&_fromAddr, sizeof(sockaddr_in));

    _toAddr.sin_family   = AF_INET;
    _toAddr.sin_port     = htons(toPort);
    _toAddr.sin_addr.s_addr = inet_addr(toIP.c_str());

    _ownAddr.sin_family = AF_INET;
    _ownAddr.sin_port   = htons(ownPort);
    _ownAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    _sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if(_sockfd < 0){
        perror("[ERROR] UDPPort::UDPPort, create socket failed\n");
        exit(-1);
    }

    setsockopt(_sockfd, SOL_SOCKET, SO_REUSEADDR, &_on, sizeof(_on));

    if(bind(_sockfd, (struct sockaddr*)&_ownAddr, sizeof(struct sockaddr)) < 0){
        perror("[ERROR] UDPPort::UDPPort, bind failed");
        exit(-1);
    }

    _sockaddrSize = sizeof(struct sockaddr);

}

UDPPort::~UDPPort(){
    close(_sockfd);
}

size_t UDPPort::send(uint8_t *sendMsg, size_t sendMsgLength){
    _sentLength = sendto(_sockfd, sendMsg, sendMsgLength, 0, (struct sockaddr*)&_toAddr, _sockaddrSize);
    if(_sentLength != sendMsgLength){
        std::cout << "[WARNING] UDPPort::send, sent " << _sentLength << " bytes, but not " << sendMsgLength << " bytes, " << strerror(errno) << std::endl;
    }
    return _sentLength;
}

size_t UDPPort::recv(uint8_t *recvMsg){
    if(_blockYN == BlockYN::NO){
        return _recvUnBlock(recvMsg, _recvLength);
    }else{
        return _recvBlock(recvMsg, _recvLength);
    }
}

size_t UDPPort::recv(uint8_t *recvMsg, size_t recvLength){
    if(_blockYN == BlockYN::NO){
        return _recvUnBlock(recvMsg, recvLength);
    }else{
        return _recvBlock(recvMsg, recvLength);
    }
}

/* block, until received */
size_t UDPPort::_recvBlock(uint8_t *recvMsg, size_t recvLength){
    size_t receivedLength = recvfrom(_sockfd, recvMsg, recvLength, MSG_WAITALL, (struct sockaddr*)&_fromAddr, &_sockaddrSize);
    if(receivedLength != recvLength){
        std::cout << "[WARNING] UDPPort::recv, block version, received " << receivedLength << " bytes, but not " << recvLength << " bytes, " << strerror(errno) << std::endl;
    }
    tcflush(_sockfd, TCIOFLUSH);
    return receivedLength;
}

/* unblock, has time out, time unit is us */
size_t UDPPort::_recvUnBlock(uint8_t *recvMsg, size_t recvLength){
    size_t receivedLength;
    /* init every time */
    FD_ZERO(&_rSet);
    FD_SET(_sockfd, &_rSet);
    _timeout = _timeoutSaved;
    
    switch ( select(_sockfd+1, &_rSet, NULL, NULL, &_timeout) ){
    case -1:
        std::cout << "[WARNING] UDPPort::recv, unblock version, receive error" << std::endl;
        return 0;
    case 0:
        if((++_isDisConnectCnt) > 200){
            isDisConnect = true;
        }
        std::cout << "[WARNING] UDPPort::recv, unblock version, wait time out" << std::endl;
        return 0;
    default:
        _isDisConnectCnt = 0;
        if(isDisConnect){
            
        }
        isDisConnect = false;

        receivedLength = recvfrom(_sockfd, recvMsg, recvLength, 0, (struct sockaddr*)&_fromAddr, &_sockaddrSize);
        if(receivedLength != recvLength){
            std::cout << "[WARNING] UDPPort::recv, unblock version, received " << receivedLength << " bytes, but not " << recvLength << " bytes, " << strerror(errno) << std::endl;
            // std::cout << "IP: " << inet_ntoa(_fromAddr.sin_addr) << std::endl;
        }

        tcflush(_sockfd, TCIOFLUSH);
        return receivedLength;
    }
}