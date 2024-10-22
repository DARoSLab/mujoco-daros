/*!
 * @file HardwareBridge.h
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */

#ifndef Pat_HARDWAREBRIDGE_H
#define Pat_HARDWAREBRIDGE_H

#ifdef __linux

#include <string>
#include <lcm-cpp.hpp>
#include <lord_imu/LordImu.h>

#include <HardwareBridge.hpp>
#include "microstrain_lcmt.hpp"

#include <common/rt/rt_can_read.h>
#include <common/rt/rt_can_write.h>
#include <common/rt/rt_xsr.h>



#include <common/rt/rt_vectornav.h>
#include "PatSystem.hpp"
#include "PatMocapManager.hpp"
#include <estimators/estimators.h>


class PatHardwareBridge : public HardwareBridge{
 public:
  PatHardwareBridge(System<float>* system);
  virtual ~PatHardwareBridge(){}

  virtual void run();

  void initHardware();
  void runMicrostrain();
  void runCANWrite();
  void runCANRead();
  void runMocap();
  void runSystem();
  void runXSR();
  void logMicrostrain();
  void systemRendering();

  void abort(const std::string& reason);
  void abort(const char* reason);

 protected:
  unsigned long long _iter_run_system = 0;
  //FBModelState<float> _state;
  can_data_lcmt _canData;
  can_command_lcmt _canCommand;

  lcm::LCM _canLCM;


  VectorNavData _vectorNavData;
  MOCAPData<float> _mocapData_est;
  MoCapData<float>* _mocapData;

  LocalizationData _localizationData;
  #ifdef MOCAP_BUILD
  PatMocapManager<float>* _mocap_manager;
  #endif
  lcm::LCM _microstrainLcm;
  std::thread _microstrainThread;
  LordImu _microstrainImu;
  microstrain_lcmt _microstrainData;
  bool _microstrainInit = false;
  bool _load_parameters_from_file;
  int _iter = 0;

  xsr_lcmt _xsrData;
};

#endif // END of #ifdef __linux
#endif  // PROJECT_HARDWARETESTBRIDGE_H
