#ifndef _UNITREE_ARM_JOYSTICK_H_
#define _UNITREE_ARM_JOYSTICK_H_

#include <vector>
#include "unitree_arm_sdk/udp.h"
#include "unitree_arm_sdk/cmdPanel.h"
#include "unitree_arm_sdk/common/joystick_common.h"

#define CTRL_BY_ALIENGO_JOYSTICK

#ifdef CTRL_BY_ALIENGO_JOYSTICK
#include "unitree_arm_sdk/common/aliengo_common.h"
#else
#include "unitree_arm_sdk/common/b1_common.h"
#endif


using namespace UNITREE_LEGGED_SDK;

class UnitreeJoystick : public CmdPanel{
public:
    UnitreeJoystick(std::vector<KeyAction*> events, 
        EmptyAction emptyAction, size_t channelNum = 1,
        double dt = 0.002);
    ~UnitreeJoystick();
    SendCmd getSendCmd(){return _sendCmd;};
private:
    SendCmd _sendCmd;
    void _read();
    void _extractCmd();

    xRockerBtnDataStruct _keyData;
    UDPPort *_udp;
    HighCmd _udpCmd;
    HighState _udpState;
};

#endif  // _UNITREE_ARM_JOYSTICK_H_
