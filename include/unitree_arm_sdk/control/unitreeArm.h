#ifndef __UNITREEARM_H
#define __UNITREEARM_H

#include "unitree_arm_sdk/control/ctrlComponents.h"

namespace UNITREE_ARM {

class unitreeArm{
public:
unitreeArm(bool hasUnitreeGripper);
~unitreeArm();
/*
 * Function: Change z1_ctrl state to fsm, wait until change complete
 * Input:    ArmFSMState
 * Output:   Whether swtich to fsm correctly
 * Note:     eaxmple: Only State_Passive could switch to State_LowCmd
 */
bool setFsm(ArmFSMState fsm);
/*
 * Function: Move arm to home position
 *           wait until arrival home position, and then switch to State_JointCtrl
 * Input:    None
 * Output:   None
 */
void backToStart();
/*
 * Function: Move arm to label position
 *           wait until arrival label position, and then switch to State_JointCtrl
 * Input:    label
 *           which should exist in z1_controller/config/saveArmStates.csv.
 *           The number of characters in label cannot be greater than 10.(char name[10])
 * Output:   None
 */
void labelRun(std::string label);
/*
 * Function: Save current position as a label to saveArmStates.csv
 *           Switch to State_JointCtrl when done
 * Input:    label
 *           name to save, which shouldn't exist in z1_controller/config/saveArmStates.csv.
 *           The number of characters in label cannot be greater than 10.(char name[10])
 * Output:   None
 */
void labelSave(std::string label);
/*
 * Function: Save current position as a label to saveArmStates.csv
 *           Switch to State_JointCtrl when done
 * Input:    label
 *           name to save, which shouldn't exist in z1_controller/config/saveArmStates.csv.
 *           The number of characters in label cannot be greater than 10.(char name[10])
 * Output:   None
 */
void teach(std::string label);
/*
 * Function: Switch to State_Teach
 * Input:    label
 *           Teach trajectory will be save as Traj_label.csv in directory z1_controller/config/
 *           The number of characters in label cannot be greater than 10.(char name[10])
 * Output:   None
 */
void teachRepeat(std::string label);
/*
 * Function: Calibrate the motor, make current position as home position
 * Input:    None
 * Output:   None
 */
void calibration();
/*
 * Function: Move the robot in a joint path
 * Input:    posture: target position, (roll pitch yaw x y z), unit: meter
 *           maxSpeed: the maximum joint speed when robot is moving, unit: radian/s
 *             range:[0, pi]
 * Output:   None
 */
bool MoveJ(Vec6 posture, double maxSpeed);
/*
 * Function: Move the robot in a joint path, and control the gripper at the same time
 * Input:    posture: target position, (roll pitch yaw x y z), unit: meter
 *           gripperPos: target angular
 *             uint: radian
 *             range:[-pi/2, 0]
 *           maxSpeed: the maximum joint speed when robot is moving
 *             unit: radian/s
 *             range:[0, pi]
 * Output:   whether posture has inverse kinematics
 */
bool MoveJ(Vec6 posture, double gripperPos, double maxSpeed);
/*
 * Function: Move the robot in a linear path
 * Input:    posture: target position, (roll pitch yaw x y z), unit: meter
 *           maxSpeed: the maximum joint speed when robot is moving, unit: m/s
 * Output:   whether posture has inverse kinematics
 */
bool MoveL(Vec6 posture, double maxSpeed);
/*
 * Function: Move the robot in a linear path, and control the gripper at the same time
 * Input:    posture: target position, (roll pitch yaw x y z), unit: meter
 *           gripperPos: target angular, uint: radian
 *             range:[-pi/2, 0]
 *           maxSpeed: the maximum joint speed when robot is moving, unit: m/s
 * Output:   whether posture has inverse kinematics
 */
bool MoveL(Vec6 posture, double gripperPos, double maxSpeed);
/*
 * Function: Move the robot in a circular path
 * Input:    middle posture: determine the shape of the circular path
 *           endPosture: target position, (roll pitch yaw x y z), unit: meter
 *           maxSpeed: the maximum joint speed when robot is moving, unit: m/s
 * Output:   whether posture has inverse kinematics
 */
bool MoveC(Vec6 middlePosutre, Vec6 endPosture, double maxSpeed);
/*
 * Function: Move the robot in a circular path, and control the gripper at the same time
 * Input:    middle posture: determine the shape of the circular path
 *           endPosture: target position, (roll pitch yaw x y z), unit: meter
 *           gripperPos: target angular, uint: radian
 *             range:[-pi/2, 0]
 *           maxSpeed: the maximum joint speed when robot is moving, unit: m/s
 * Output:   whether posture has inverse kinematics
 */
bool MoveC(Vec6 middlePosutre, Vec6 endPosture, double gripperPos, double maxSpeed);
/*
 * Function: Control robot with q&qd command  in joint space or posture command in cartesian space
 * Input:    fsm: ArmFSMState::JOINTCTRL or ArmFSMState::CARTESIAN
 * Output:   whether posture has inverse kinematics
 * Description: 1. ArmFSMState::JOINTCTRL, 
 *              if you run function startTrack(ArmFSMState::JOINTCTRL), 
 *              firstly, the following parameters will be set at the first time:
 *                  q : <---- lowstate->getQ()
 *                  qd: <---- lowstate->getQd()
 *                  gripperQ:  <----   lowstate->getGripperQ()
 *                  gripperQd: <----  lowstate->getGripperQd()
 *              then you can change these parameters to control robot
 *              2. ArmFSMState::CARTESIAN, 
 *              if you run function startTrack(ArmFSMState::JOINTCTRL), 
 *              firstly, the following parameters will be set at the first time:
 *                  endPosture: <---- lowstate->endPosture;
 *                  gripperQ:   <---- lowstate->getGripperQ()
 *              then you can change these parameters to control robot
 */
void startTrack(ArmFSMState fsm);
/*
 * Function: send udp message to z1_ctrl and receive udp message from it
 * Input:    None
 * Output:   None
 * Description: sendRecvThread will run sendRecv() at a frequency of 500Hz
 *              ctrlcomp.sendRecv() is called in unitreeArm.sendRecv(),
 *              and set command parameters in unitreeArm to lowcmd automatically
 *              If you want to control robot under JOINTCTRL, CARTESIAN or LOWCMD,
 *              instead of MovecJ, MoveL, MoveC, and so on
 *              it is recommended to create your own thread to process command parameters
 *              (see stratTrack() description)
 *              and execute sendRecv() at the end of thread
 */
void sendRecv();

/*
 * Function: whether to wait for the command to finish
 * Input:    true or false
 * Output:   None
 * Description: For example, MoveJ will send a trajectory command to z1_controller and then
 *              run usleep() to wait for the trajectory execution to complete.
 *              If set [wait] to false, MoveJ  will send command only and user should judge 
 *              for youself whether the command is complete.
 *                  Method 1: if(_ctrlComp->recvState.state != fsm)
 *                      After trajectory complete, the FSM will switch to ArmFSMState::JOINTCTRL
 *                      automatically
 *                  Method 2: if((lowState->endPosture - endPostureGoal).norm() < error)
 *                      Check whether current posture reaches the target
 *              Related functions:
 *                  MoveJ(), MoveL(), MoveC(), backToStart(), labelRun(), teachRepeat()
 */
void setWait(bool Y_N);

//command parameters
Vec6 q, qd, tau;
Vec6 endPosture;
double gripperQ, gripperW, gripperTau;

LoopFunc *sendRecvThread;
LowlevelCmd *lowcmd;//same as _ctrlComp->lowcmd
LowlevelState *lowstate;//same as _ctrlComp->lowstate
CtrlComponents *_ctrlComp;

private:
    bool _isWait = true;
};

}
#endif
