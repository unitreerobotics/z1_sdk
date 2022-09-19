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
    SETTRAJ,
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

enum class TrajType{
    MoveJ,
    MoveL,
    MoveC,
    Stop
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

//140bytes
union UDPSendCmd{
    uint8_t checkCmd;
    JointCmd jointCmd[7];
};


// 16*7=112 Byte
union UDPRecvState{
    JointState jointState[7];
    uint8_t errorCheck[16];
};

struct Posture{
    double roll;
    double pitch;
    double yaw;
    double x;
    double y;
    double z;
};

struct TrajCmd{
    TrajType trajType;
    Posture posture[2];
    double gripperPos;
    double maxSpeed;
    double stopTime;
    int trajOrder;
};

union ValueUnion{
    char name[10];
    JointCmd jointCmd[7];
    TrajCmd trajCmd;
};

struct SendCmd{
    uint8_t head[2];
    ArmFSMState state;
    ArmFSMValue value;
    bool track;// whether let arm track jointCmd in State_JOINTCTRL or posture[0] in State_CARTESIAN
    ValueUnion valueUnion;
};

struct RecvState{
    uint8_t head[2];
    ArmFSMState state;
    JointState jointState[7];
    Posture cartesianState;
};

#pragma pack()

#endif  // _UNITREE_ARM_ARM_MSG_H_