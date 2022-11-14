#include "control/unitreeArm.h"

class JointTraj{
public:
    JointTraj(){};
    ~JointTraj(){};
    void setJointTraj(Vec6 startQ, Vec6 endQ, double speed){
        _startQ = startQ;
        _endQ = endQ;
        Vec6 deltaQ = endQ - startQ;
        Vec6 t = deltaQ / speed;
        for(int i(0); i<6; i++){
            _pathTime = fabs(t(i)) > _pathTime ? fabs(t(i)) : _pathTime;
        }
        _generateA345();
    };
    void setGripper(double startQ, double endQ, double speed){
        _gripperStartQ = startQ;
        _gripperEndQ = endQ; 
        double t = fabs(endQ - startQ)/speed;
        _pathTime = t > _pathTime ? t : _pathTime;
        _generateA345();
    }
    bool getJointCmd(Vec6& q, Vec6& qd, double& gripperQ, double& gripperQd){
        _runTime();

        if(!_reached){
            _s = _a3*pow(_tCost,3) + _a4*pow(_tCost,4) + _a5*pow(_tCost,5);
            _sDot = 3*_a3*pow(_tCost,2) + 4*_a4*pow(_tCost,3) + 5*_a5*pow(_tCost,4);
            q = _startQ + _s*(_endQ - _startQ);
            qd = _sDot * (_endQ - _startQ);

            gripperQ  = _gripperStartQ + (_gripperEndQ-_gripperStartQ)*_s;
            gripperQd = (_gripperEndQ-_gripperStartQ) * _sDot;
        }else{
            q = _endQ;
            qd.setZero();
        }

        return _reached;
    }
private:
    //joint trajectory parameters
    Vec6 _startQ, _endQ;
    double _gripperStartQ, _gripperEndQ;
    double _pathTime, _tCost, _currentTime, _startTime;
    double _a3, _a4, _a5;
    double _s, _sDot;       // [0, 1]
    bool _pathStarted = false;
    bool _reached = false;

    void _runTime(){
        _currentTime = getTimeSecond();

        if(!_pathStarted){
            _pathStarted = true;
            _startTime = _currentTime;
            _tCost = 0;
        }
        _tCost = _currentTime - _startTime;
        _reached = (_tCost>_pathTime) ? true : false;
        _tCost = (_tCost>_pathTime) ? _pathTime : _tCost;
    }
    void _generateA345(){
        if(_pathTime == 0){
            _a3 = 0;
            _a4 = 0;
            _a5 = 0;
        }else{
            _a3 =  10 / pow(_pathTime, 3);   // parameters of path
            _a4 = -15 / pow(_pathTime, 4);
            _a5 =   6 / pow(_pathTime, 5);
        }
    }
};


class Z1ARM : public unitreeArm{
public:
    Z1ARM(CtrlComponents * ctrlComp):unitreeArm(ctrlComp){
        myThread = new LoopFunc("Z1Communication", _ctrlComp->dt, boost::bind(&Z1ARM::run, this));
    };
    ~Z1ARM(){delete myThread;};
    void setJointTraj(Vec6 startQ,Vec6 endQ, double speed){
    };
    void run(){
        if(trajStart){
            if(jointTraj.getJointCmd(q, qd, gripperQ, gripperW)){
                trajStart = false;
            }
        }
        // std::cout << q.transpose() <<" "<< gripperQ <<std::endl;
        sendRecv();
    }

    JointTraj jointTraj;
    bool trajStart = false;
    LoopFunc *myThread;
};


int main(){
    std::cout << std::fixed << std::setprecision(3);
    CtrlComponents *ctrlComp = new CtrlComponents(0.002);
    Z1ARM arm(ctrlComp);
    arm.myThread->start();

    Vec6 forward;
    forward << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0;
    double speed = 0.5;
    arm.startTrack(ArmFSMState::JOINTCTRL);
    arm.jointTraj.setJointTraj(arm._ctrlComp->lowstate->getQ(), forward, speed);
    arm.jointTraj.setGripper(arm._ctrlComp->lowstate->getGripperQ(), -1, speed);
    
    arm.trajStart = true;
    while (arm.trajStart){
        sleep(1);
    }
    arm.backToStart();
    arm.setFsm(ArmFSMState::PASSIVE);
    arm.myThread->shutdown();
    delete ctrlComp;
    return 0;
}