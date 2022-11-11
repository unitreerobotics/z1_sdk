#ifndef _UNITREE_ARM_TIMER_H_
#define _UNITREE_ARM_TIMER_H_

#include <iostream>
#include <unistd.h>
#include <stdint.h>
#include <sys/time.h>
#include <sys/timerfd.h>

//时间戳  微秒级， 需要#include <sys/time.h> 
inline long long getSystemTime(){
    struct timeval t;  
    gettimeofday(&t, NULL);
    return 1000000 * t.tv_sec + t.tv_usec;  
}
//时间戳  秒级， 需要getSystemTime()
inline double getTimeSecond(){
    double time = getSystemTime() * 0.000001;
    return time;
}
/*
等待函数，微秒级，从startTime开始等待waitTime微秒
目前不推荐使用，建议使用AbsoluteTimer类
*/
inline void absoluteWait(long long startTime, long long waitTime){
    if(getSystemTime() - startTime > waitTime){
        std::cout << "[WARNING] The waitTime=" << waitTime << " of function absoluteWait is not enough!" << std::endl
        << "The program has already cost " << getSystemTime() - startTime << "us." << std::endl;
    }
    while(getSystemTime() - startTime < waitTime){
        usleep(50);
    }
}


/*
waitTimeS = 0 means do not care time out
*/
class AbsoluteTimer{
public:
    AbsoluteTimer(double waitTimeS);
    ~AbsoluteTimer();
    void start();
    bool wait();
private:
    void _updateWaitTime(double waitTimeS);
    int _timerFd;
    uint64_t _missed;
    double _waitTime;
    double _startTime;
    double _leftTime;
    double _nextWaitTime;
    itimerspec _timerSpec;
};

#endif