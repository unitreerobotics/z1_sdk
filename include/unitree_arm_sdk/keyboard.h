#ifndef _UNITREE_ARM_KEYBOARD_H_
#define _UNITREE_ARM_KEYBOARD_H_

#include <string>
#include <vector>
#include <deque>

#include "unitree_arm_sdk/udp.h"
#include "unitree_arm_sdk/common/arm_common.h"
#include "unitree_arm_sdk/cmdPanel.h"

class Keyboard : public CmdPanel{
public:
    Keyboard(std::vector<KeyAction*> events, 
        EmptyAction emptyAction, size_t channelNum = 1, double dt = 0.002);
    ~Keyboard();
    std::string getString(std::string slogan);
    std::vector<double> stringToArray(std::string slogan);
    std::vector<std::vector<double> > stringToMatrix(std::string slogan);
private:
    void _read();
    void _pauseKey();
    void _startKey();
    void _extractCmd();

    fd_set _set;
    char _c = '\0';

    termios _oldSettings;
    termios _newSettings;
    timeval _tv;
};


class UnitreeKeyboardUDPSend : public CmdPanel{
public:
    UnitreeKeyboardUDPSend(std::vector<KeyAction*> events, 
        EmptyAction emptyAction, size_t channelNum = 1,
        double dt = 0.002);
    ~UnitreeKeyboardUDPSend();
    std::string getString(std::string slogan);
    std::vector<double> stringToArray(std::string slogan);
    std::vector<std::vector<double> > stringToMatrix(std::string slogan);
    
private:
    // keyboard communication
    void _read();
    void _pauseKey();
    void _startKey();
    void _extractCmd();

    fd_set _set;
    char _c = '\0';

    termios _oldSettings;
    termios _newSettings;
    timeval _tv;

    // udp communication
    void _communicationUDP();
    void _extractCmdKeyboard();

    UDPPort *_udp;
    SendCmd _sendCmd;
    RecvState _recvState;
    std::vector<std::vector<double>> posture;
    std::string slogan;
};


class UnitreeKeyboardUDPRecv : public CmdPanel{
public:
    UnitreeKeyboardUDPRecv(std::vector<KeyAction*> events, 
        EmptyAction emptyAction, size_t channelNum = 1,
        double dt = 0.002);
    ~UnitreeKeyboardUDPRecv();
    
    SendCmd getSendCmd(){return _sendCmd;};
    void getRecvState(RecvState recvState){
        _recvState = recvState;
    };

private:
    void _read();
    void _extractCmd();

    UDPPort *_udp;
    SendCmd _sendCmd;
    RecvState _recvState;
};

#endif // _UNITREE_ARM_KEYBOARD_H_