#ifndef _UNITREE_ARM_ARM_COMMON_H_
#define _UNITREE_ARM_ARM_COMMON_H_

#include <stdint.h>

#pragma pack(1)

// 4 Byte
enum class ArmFSMState{
    INVALID,
    PASSIVE,
    JOINTCTRL,
    CARTESIAN,
    MOVEJ,
    MOVEL,
    MOVEC,
    TRAJECTORY,
    TOSTATE,
    SAVESTATE,
    TEACH,
    TEACHREPEAT,
    CALIBRATION,
    DANCE00,
    DANCE01,
    DANCE02,
    DANCE03,
    DANCE04,
    DANCE05,
    DANCE06,
    DANCE07,
    DANCE08,
    DANCE09,
    BACKTOSTART,
    GRIPPER_OPEN,
    GRIPPER_CLOSE,
    NEXT,
    LOWCMD
};

// 4 Byte
enum class ArmFSMValue{
    INVALID,
    Q,A,
    W,S,  
    E,D,
    R,F,
    T,G, 
    Y,H,   
    DOWN,
    UP
};

// 20 Byte
struct JointCmd{
    float T;
    float W;
    float Pos;
    float K_P;
    float K_W;
};

// 16 Byte
struct JointState{
    float T;
    float W;
    float Acc;
    float Pos;
};

// 140 Byte
union UDPSendCmd{
    uint8_t checkCmd;
    JointCmd jointCmd[7];
};

// 16*7=112 Byte
union UDPRecvState{
    uint8_t singleState;
    uint8_t selfCheck[10];
    JointState jointState[7];
    uint8_t errorCheck[16];
};

// 24 Byte
struct Posture{
    float roll;
    float pitch;
    float yaw;
    float x;
    float y;
    float z;
};

// 48 Byte
struct MoveC{
    Posture middlePosture;
    Posture endPosture;
};

// 20*7=140 Byte
union ValueUnion{
    Posture moveJ;
    Posture moveL;
    Posture moveC[2];
    char teach[10];
    char teachRepeat[10];
    char saveState[10];
    char toState[10];
    JointCmd jointCmd[7];
};

// 2+4+4+140 = 150 Byte
struct SendCmd{
    uint8_t head[2];
    ArmFSMState state;
    ArmFSMValue value;
    ValueUnion valueUnion;
};

// 2+4+112+24 = 142 Byte
struct RecvState{
    uint8_t head[2];
    ArmFSMState state;
    JointState jointState[7];
    Posture cartesianState;
};

#pragma pack()

#endif  // _UNITREE_ARM_ARM_MSG_H_