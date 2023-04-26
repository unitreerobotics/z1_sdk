#ifndef _UNITREE_ARM_LOOP_H_
#define _UNITREE_ARM_LOOP_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <pthread.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "unitree_arm_sdk/utilities/timer.h"

namespace UNITREE_ARM {
typedef boost::function<void ()> Callback;

class Loop {
public:
  Loop(std::string name, float period, int bindCPU = -1);
  ~Loop();
  void start();
  void shutdown();
  virtual void functionCB() = 0;

private:
  void entryFunc();

  std::string _name;
  float _period;
  int _bindCPU;
  bool _bind_cpu_flag = false;
  bool _isrunning = false;
  std::thread _thread;

  size_t _runTimes = 0;
  size_t _timeOutTimes = 0;

  AbsoluteTimer *_timer;
};

class LoopFunc : public Loop {
public:
/*
 * Function: create a thead run once every period
 * Input: name: indicate what the thread aims to
 *        period : time, unit: second
 *        _cb : the function pointer
 */
  LoopFunc(std::string name, float period, const Callback& _cb)
    : Loop(name, period), _fp(_cb){}
  LoopFunc(std::string name, float period, int bindCPU, const Callback& _cb)
    : Loop(name, period, bindCPU), _fp(_cb){}
  void functionCB() { (_fp)(); }
private:
  boost::function<void ()>  _fp;
};
}
#endif  // _UNITREE_ARM_LOOP_H_