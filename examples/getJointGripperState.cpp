#include "unitreeArm.h"

class Custom : public unitreeArm {
public:
    Custom(){};
    ~Custom(){};
    void printJointGripperState();
    Posture _gripperState;
};

void Custom::printJointGripperState() {
    getJointState(_jointState);
    std::cout<<std::endl;
    std::cout<<"------ joint Pos ------"<<std::endl;
    std::cout<<"joint0: "<<_jointState[0].Pos<<std::endl;
    std::cout<<"joint1: "<<_jointState[1].Pos<<std::endl;
    std::cout<<"joint2: "<<_jointState[2].Pos<<std::endl;
    std::cout<<"joint3: "<<_jointState[3].Pos<<std::endl;
    std::cout<<"joint4: "<<_jointState[4].Pos<<std::endl;
    std::cout<<"joint5: "<<_jointState[5].Pos<<std::endl;
    std::cout<<"------ joint Pos ------"<<std::endl;
    std::cout<<std::endl;

    getGripperState(_gripperState);
    std::cout<<std::endl;
    std::cout<<"------ GripperState ------"<<std::endl;
    std::cout<<"roll: "<<_gripperState.roll<<std::endl;
    std::cout<<"pitch: "<<_gripperState.pitch<<std::endl;
    std::cout<<"yaw: "<<_gripperState.yaw<<std::endl;
    std::cout<<"x: "<<_gripperState.x<<std::endl;
    std::cout<<"y: "<<_gripperState.y<<std::endl;
    std::cout<<"z: "<<_gripperState.z<<std::endl;
    std::cout<<"------ GripperState Pos ------"<<std::endl;
    std::cout<<std::endl;
  
}
 
int main() {

    Custom custom;
    LoopFunc loop_udpSendRecv("udp", 0.002, boost::bind(&Custom::UDPSendRecv,&custom));
    LoopFunc loop_printJointState("printJointState", 1, boost::bind(&Custom::printJointGripperState,&custom));
    loop_udpSendRecv.start();
    loop_printJointState.start();
    
    while (1) {
        
        usleep(1000000);
    }
}