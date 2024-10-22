#ifndef __HARDWARE_BRIDGE__
#define __HARDWARE_BRIDGE__

#ifdef __linux

#include <System.hpp>
#include <Utilities/PeriodicTask.h>
#include "gamepad_lcmt.hpp"

#include <common/rt/rt_sbus.h>
#include <common/rt/rt_rc_interface.h>

#include <sys/mman.h>

#define MAX_STACK_SIZE 16384  // 16KB  of stack
#define TASK_PRIORITY 49      // linux priority, this is not the nice value

class HardwareBridge{
  public:
    HardwareBridge(System<float>* system):
      _system(system),
      statusTask(&taskManager, 0.5f){}

    virtual ~HardwareBridge(){}

    virtual void run() = 0;

  void prefaultStack(){
    printf("[Init] Prefault stack...\n");
    volatile char stack[MAX_STACK_SIZE];
    memset(const_cast<char*>(stack), 0, MAX_STACK_SIZE);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
      printf("mlockall failed.  This is likely because you didn't run robot as root\n");
    } 
  }
  void setupScheduler(){
    printf("[Init] Setup RT Scheduler...\n");
    struct sched_param params;
    params.sched_priority = TASK_PRIORITY;
    if (sched_setscheduler(0, SCHED_FIFO, &params) == -1) {
      printf("sched_setscheduler failed.\n");
    }
  }
  void initCommon(){
    prefaultStack();
    setupScheduler();
    //if (!_interfaceLCM.good()) { printf("_interfaceLCM failed to initia\n");   }

    //_interfaceLCM.subscribe("interface", &QuadrupedHardwareBridge::handleGamepadLCM, this);
    //_interfaceLCM.subscribe("interface_request",
        //&QuadrupedHardwareBridge::handleControlParameter, this);
    //_interfaceLcmThread = std::thread(&QuadrupedHardwareBridge::handleInterfaceLCM, this);
  }
  //void handleGamepadLCM(const lcm::ReceiveBuffer* rbuf, const std::string& chan,
                        //const gamepad_lcmt* msg);

  //void handleInterfaceLCM();
  //void handleControlParameter(const lcm::ReceiveBuffer* rbuf,
                              //const std::string& chan,
                              //const control_parameter_request_lcmt* msg);

  void run_sbus(){
    if (_port > 0) { 
      int x = receive_sbus(_port);
      if (x) { sbus_packet_complete();  }
    }
  }


  protected:
    System<float>* _system = nullptr;
    PrintTaskStatus statusTask;

    PeriodicTaskManager taskManager;
    //GamepadCommand _gamepadCommand;

    //lcm::LCM _interfaceLCM;
    //lcm::LCM _visualizationLCM;
    //control_parameter_response_lcmt _parameter_response_lcmt;

    //SpiData _spiData;
    //SpiCommand _spiCommand;

    //TiBoardCommand _tiBoardCommand[4];
    //TiBoardData _tiBoardData[4];

    bool _firstRun = true;
    //RobotControlParameters _robotParams;
    u64 _iterations = 0;
    //std::thread _interfaceLcmThread;
    //volatile bool _interfaceLcmQuit = false;
    //ControlParameters* _userControlParameters = nullptr;

    int _port;
};

#endif // END of #ifdef __linux
#endif
