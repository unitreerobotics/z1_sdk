#include <math.h>
#include "utilities/timer.h"

AbsoluteTimer::AbsoluteTimer(double waitTimeS)
    :_waitTime(waitTimeS){
    _timerFd = timerfd_create(CLOCK_MONOTONIC, 0);
}

AbsoluteTimer::~AbsoluteTimer(){
    close(_timerFd);
}

void AbsoluteTimer::start(){
    _startTime = getTimeSecond();
}

bool AbsoluteTimer::wait(){
    if(_waitTime == 0.0){
        return true;
    }
    _leftTime = _waitTime - (getTimeSecond() - _startTime);
    if(_leftTime < 0.0){
        // std::cout << "[WARNING] The wait time " << _waitTime * 1000.0 << "ms of AbsoluteTimer is not enough!" << std::endl
        // << "The program has already cost " << (getTimeSecond() - _startTime) * 1000.0 << "ms." << std::endl;
        return false;
    }

    int m;
    while(true){
        _leftTime = _waitTime - (getTimeSecond() - _startTime);
        if(_leftTime < 1e-6){
            start();
            return true;
        }
        else if(_leftTime < 2e-3){
            _nextWaitTime = _leftTime / 2.0;
        }
        else{
            _nextWaitTime = 2e-3;
        }
        _updateWaitTime(_nextWaitTime);
        m = read(_timerFd, &_missed, sizeof(_missed));
    }

    return false;
}

void AbsoluteTimer::_updateWaitTime(double waitTimeS){
    int seconds = (int)waitTimeS;
    int nanoseconds = (int)(1e9 * fmod(waitTimeS, 1.f));

    _timerSpec.it_interval.tv_sec = seconds;
    _timerSpec.it_interval.tv_nsec = nanoseconds;
    _timerSpec.it_value.tv_sec = seconds;
    _timerSpec.it_value.tv_nsec = nanoseconds;

    timerfd_settime(_timerFd, 0, &_timerSpec, nullptr);
}