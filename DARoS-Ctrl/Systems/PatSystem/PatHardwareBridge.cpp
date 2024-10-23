/*!
 * @file HardwareBridge.cpp
 * @brief Interface between robot code and robot hardware
 *
 * This class initializes the hardware of both robots and allows the robot
 * controller to access it
 */
#ifdef __linux

#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <Configuration.h>

#include "PatHardwareBridge.hpp"
#include <pretty_print.h>


PatHardwareBridge::PatHardwareBridge(System<float>* system)
    : HardwareBridge(system),
    _canLCM(getLcmUrl(255)),
    _microstrainLcm(getLcmUrl(255))
{
  // #ifdef MOCAP_BUILD
  // _mocap_manager = new PatMocapManager<float>();
  // #endif
}

/*!
 * Main method for Mini Cheetah hardware
 */
void PatHardwareBridge::run() {
  initCommon();
  initHardware();
  PatSystem<float>* pat_sys = dynamic_cast<PatSystem<float> * >(_system);
  pat_sys->_sys_canData = &_canData;
  pat_sys->_sys_canCommand = &_canCommand;
  pat_sys->_sys_vectorNavData = &_vectorNavData;
  pat_sys->_sys_mocapData = &_mocapData_est;
  pat_sys->_sys_localizationData = &_localizationData;
  pat_sys->_mocapData = _mocapData;
  pat_sys->initialization();


  // init control thread
  statusTask.start();

  // CAN Write Task
  PeriodicMemberFunction<PatHardwareBridge> CANWriteTask(
      &taskManager, 0.002, "CAN_write", &PatHardwareBridge::runCANWrite, this);
  CANWriteTask.start();

  // CAN Read Task
  PeriodicMemberFunction<PatHardwareBridge> CANReadTask(
      &taskManager, 0.00002, "CAN_read", &PatHardwareBridge::runCANRead, this);
  CANReadTask.start();

  // MOCAP Task
  #ifdef MOCAP_BUILD
  PeriodicMemberFunction<PatHardwareBridge> MoCapTask(
      &taskManager, 0.000001, "MoCap", &PatHardwareBridge::runMocap, this);
  MoCapTask.start();
  #endif

  //System
  PeriodicMemberFunction<PatHardwareBridge> systemTask(
      &taskManager, pat_sys->getParameters().controller_dt,
      "robot-control", &PatHardwareBridge::runSystem, this);
  systemTask.start();
  //System Rendering
  PeriodicMemberFunction<PatHardwareBridge> system_rendering_Task(
      &taskManager, 0.01, "system-rendering", &PatHardwareBridge::systemRendering, this);
  system_rendering_Task.start();
  //XSR-SIM DONGLE RC controller Task
  PeriodicMemberFunction<PatHardwareBridge> xsrTask(
        &taskManager, .001, "rc_controller", &PatHardwareBridge::runXSR, this);// 0.005
    xsrTask.start();

  for (;;) {
    usleep(1000000);
  }
}

void PatHardwareBridge::runXSR(){
  run_xsr();
  xsr_complete();
}
void PatHardwareBridge::runMocap(){
  #ifdef MOCAP_BUILD
  _mocap_manager->run();
  _mocapData->com_vel = _mocap_manager->_body_velocity_est;
  _mocapData->com_pos = _mocap_manager->_body_position_est;
  #endif

}

/*!
 * Initialize Patroclus specific hardware
 */
void PatHardwareBridge::initHardware() {
  #ifdef MOCAP_BUILD
    _mocap_manager = new PatMocapManager<float>(&_mocapData_est);
    _mocap_manager->initialize();
    usleep(4000000);
  #endif
  _vectorNavData.quat << 1, 0, 0, 0;
  printf("[PatHardware] Init vectornav\n");
  if (!init_vectornav(&_vectorNavData)) {
    printf("Vectornav failed to initialize\n");
    //initError("failed to initialize vectornav!\n", false);
  }
  _mocapData = new MoCapData<float>();
  _mocapData->com_pos<<0, 0, 0;
  _mocapData->com_vel<<0, 0, 0;
  _mocapData->marker_orientation<< 1, 0, 0, 0;
  init_can_write();
  init_can_read();
  init_xsr();

}


/*!
 * Run Pat CAN write
 */
void PatHardwareBridge::runCANWrite() {
  can_command_lcmt *cmd = get_can_command();
  memcpy(cmd, &_canCommand, sizeof(can_command_lcmt));
  can_write_run();

}
/*!
 * Run Pat CAN read
 */

void PatHardwareBridge::runCANRead() {
  can_data_lcmt *data = get_can_data();
  can_read_run();
  memcpy(&_canData, data, sizeof(can_data_lcmt));
}

void PatHardwareBridge::runSystem(){

  if ( _system->Estop() ) {
    if (_iter_run_system % 1000 == 0)   printf("ESTOP!\n");
  }else{
    _system->onestep_forward();
  }
  ++_iter_run_system;

}

void PatHardwareBridge::systemRendering(){
  _system->updateFBModelStateEstimate();
  _system->renderSystem();
}


#endif  // __linux
