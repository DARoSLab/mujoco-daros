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

#include "PatHardwareTestBridge.hpp"
#include <Utilities/pretty_print.h>

// #define USE_MICROSTRAIN
// #define PRINT_CAN_DATA
#define PRINT_XSR_DATA
PatHardwareTestBridge::PatHardwareTestBridge(System<float>* system)
    : HardwareBridge(system),
    _canLCM(getLcmUrl(255)),
    _microstrainLcm(getLcmUrl(255))
{
  //_system->getRobot()->getInitialState(_state);
  #ifdef MOCAP_BUILD
  _mocap_manager = new PatMocapManager<float>();
  #endif
}

/*!
 * Main method for Mini Cheetah hardware
 */
void PatHardwareTestBridge::run() {
  initCommon();
  initHardware();
  PatSystem<float>* pat_sys = dynamic_cast<PatSystem<float> * >(_system);
  pat_sys->_sys_canData = &_canData;
  pat_sys->_sys_canCommand = &_canCommand;
  pat_sys->_sys_vectorNavData = &_vectorNavData;
  pat_sys->_sys_localizationData = &_localizationData;
  pat_sys->_mocapData = _mocapData;

  pat_sys->initialization();


  // // init control thread
  statusTask.start();
  //
  // // can Task start
  PeriodicMemberFunction<PatHardwareTestBridge> CANWriteTask(
      &taskManager, 0.002, "CAN_write", &PatHardwareTestBridge::runCANWrite, this);
  CANWriteTask.start();

  PeriodicMemberFunction<PatHardwareTestBridge> CANReadTask(
      &taskManager, 0.00002, "CAN_read", &PatHardwareTestBridge::runCANRead, this);
  CANReadTask.start();
  #ifdef MOCAP_BUILD
  PeriodicMemberFunction<PatHardwareTestBridge> MoCapTask(
      &taskManager, 0.02, "MoCap", &PatHardwareTestBridge::runMocap, this);
  MoCapTask.start();
  #endif
  //microstrain
  // if(_microstrainInit)
  //   _microstrainThread = std::thread(&PatHardwareTestBridge::runMicrostrain, this);

  //System
  PeriodicMemberFunction<PatHardwareTestBridge> systemTask(
      &taskManager, pat_sys->getParameters().controller_dt,
      "robot-control", &PatHardwareTestBridge::runSystem, this);
  systemTask.start();
  //
  PeriodicMemberFunction<PatHardwareTestBridge> system_rendering_Task(
      &taskManager, 0.01, "system-rendering", &PatHardwareTestBridge::systemRendering, this);
  system_rendering_Task.start();

  //rc controller
  // _port = init_sbus(false, "/dev/ttyS6");  // Not Simulation
  // PeriodicMemberFunction<HardwareBridge> sbusTask(
  //     &taskManager, .001, "rc_controller", &HardwareBridge::run_sbus, this);// 0.005
  // sbusTask.start();

  PeriodicMemberFunction<PatHardwareTestBridge> xsrTask(
        &taskManager, .001, "rc_controller", &PatHardwareTestBridge::runXSR, this);// 0.005
    xsrTask.start();

  // temporary hack: microstrain logger
  //PeriodicMemberFunction<PatHardwareBridge> microstrainLogger(
      //&taskManager, .001, "microstrain-logger", &PatHardwareTestBridge::logMicrostrain, this);
  //microstrainLogger.start();


  // visualization start
  //PeriodicMemberFunction<PatHardwareBridge> visualizationLCMTask(
      //&taskManager, .0167, "lcm-vis",
      //&PatHardwareTestBridge::publishVisualizationLCM, this);
  //visualizationLCMTask.start();

  for (;;) {
    usleep(1000000);
    // printf("joy %f\n", _robotRunner->driverCommand->leftStickAnalog[0]);
  }
}

void PatHardwareTestBridge::runXSR(){
  xsr_lcmt *data = get_xsr_data();
  run_xsr();
  memcpy(&_xsrData, data, sizeof(xsr_lcmt));
  #ifdef PRINT_XSR_DATA
  std::cout << "estop: " << _xsrData.estop_switch << '\n';
  std::cout << "loc_selector_switch: " << _xsrData.loc_selector_switch << '\n';
  std::cout << "trigger_switch: " << _xsrData.trigger_switch << '\n';
  std::cout << "left_pot_x: " << _xsrData.left_x_pot<< '\n';
  std::cout << "left_y_pot: " << _xsrData.left_y_pot << '\n';
  std::cout << "right_pot_x: " << _xsrData.right_x_pot<< '\n';
  std::cout << "right_y_pot: " << _xsrData.right_y_pot << '\n';
  #endif
  xsr_complete();
}
void PatHardwareTestBridge::runMocap(){
  #ifdef MOCAP_BUILD
  _mocap_manager->run();
  _mocapData->com_vel = _mocap_manager->_body_velocity_est;
  _mocapData->com_pos = _mocap_manager->_body_position_est;
  #endif
}
void PatHardwareTestBridge::runMicrostrain() {
  while(true) {
    _microstrainImu.run();

#ifdef USE_MICROSTRAIN
    _vectorNavData.accelerometer = _microstrainImu.acc;
    _vectorNavData.quat[0] = _microstrainImu.quat[1];
    _vectorNavData.quat[1] = _microstrainImu.quat[2];
    _vectorNavData.quat[2] = _microstrainImu.quat[3];
    _vectorNavData.quat[3] = _microstrainImu.quat[0];
    _vectorNavData.gyro = _microstrainImu.gyro;
#endif
  }
}

void PatHardwareTestBridge::logMicrostrain() {
  _microstrainImu.updateLCM(&_microstrainData);
  _microstrainLcm.publish("microstrain", &_microstrainData);
}


/*!
 * Initialize Mini Cheetah specific hardware
 */
void PatHardwareTestBridge::initHardware() {
  _vectorNavData.quat << 1, 0, 0, 0;
#ifndef USE_MICROSTRAIN
  printf("[PatHardware] Init vectornav\n");
  if (!init_vectornav(&_vectorNavData)) {
    printf("Vectornav failed to initialize\n");
    //initError("failed to initialize vectornav!\n", false);
  }
#endif
#ifdef MOCAP_BUILD
  _mocap_manager->initialize();
#endif
  _mocapData = new MoCapData<float>();
  _mocapData->com_pos<<0, 0, 0;
  _mocapData->com_vel<<0, 0, 0;
  _mocapData->marker_orientation<< 1, 0, 0, 0;
  init_can_write();
  init_can_read();
  init_xsr();
  // _microstrainInit = _microstrainImu.tryInit(0, 921600);

}


/*!
 * Run Pat CAN write
 */
void PatHardwareTestBridge::runCANWrite() {
  // float curr_time(0.0f);
  // float omega(0.75f);
  // float pi(3.141592f);
  // // float amp_a(0.2f);
  // // float amp_h(0.5f);
  // // float amp_k(1.7f);
  // float amp_a(0.0f);
  // float amp_h(0.0f);
  // float amp_k(0.0f);
  // float kp(0.0);
  // float kd(0.0);
  // float p_des_a, p_des_h, p_des_k;
  // float v_des_a, v_des_h, v_des_k;
  can_command_lcmt *cmd = get_can_command();
  //
  // curr_time = 0.002*_iter++;
  // // p_des =amp*sin(2*pi*omega*curr_time);
  //
  // p_des_a = 0.0; //amp_a + amp_a*sin(2*pi*omega*curr_time-pi/2);
  // v_des_a = 0.0; //2*amp_a*pi*omega*cos(2*pi*omega*curr_time-pi/2);
  //
  // p_des_h = 0.0; //amp_h + amp_h*sin(2*pi*omega*curr_time-pi/2);
  // v_des_h = 0.0; //2*amp_h*pi*omega*cos(2*pi*omega*curr_time-pi/2);
  //
  // p_des_k = 2.9; // amp_k + amp_k*sin(2*pi*omega*curr_time-pi/2);
  // v_des_k = 0.0; //2*amp_k*pi*omega*cos(2*pi*omega*curr_time-pi/2);
  //
  // // p_des = 0.5;
  // v_des_k = 0.0*curr_time;
  // p_des_a = 0.0;
  // p_des_h = -1.0;
  // for(int leg=0; leg<2; leg++){
  //
  //   _canCommand.q_des_abad[leg]= p_des_a;
  //   _canCommand.q_des_hip[leg]= p_des_h; //pow(-1, leg+1)*p_des_h;
  //   _canCommand.q_des_knee[leg]= p_des_k;
  //   _canCommand.qd_des_abad[leg]= v_des_a; //pow(-1, leg+1)*v_des_a;
  //   _canCommand.qd_des_hip[leg]= v_des_h; //pow(-1, leg+1)*v_des_h;
  //   _canCommand.qd_des_knee[leg]= v_des_k; //pow(-1, leg)*v_des_k;
  //   _canCommand.kp_abad[leg]= kp;
  //   _canCommand.kp_hip[leg]= kp;
  //   _canCommand.kp_knee[leg]= kp;
  //   _canCommand.kd_abad[leg]= kd;
  //   _canCommand.kd_hip[leg]= kd;
  //   _canCommand.kd_knee[leg]= kd;
  //   _canCommand.tau_abad_ff[leg]=0.0;
  //   _canCommand.tau_hip_ff[leg]=0.0;
  //   _canCommand.tau_knee_ff[leg]=0.0;
  //
  // }
  // _canCommand.q_des_abad[0] = 0.0;
  // _canCommand.q_des_abad[1] = 0.0;
  //
  // _canCommand.qd_des_abad[0] = 0.0;
  // _canCommand.qd_des_abad[1] = 0.0;
  //
  // _canCommand.q_des_hip[0] = -0.40;
  // _canCommand.q_des_hip[1] = -0.3;
  //
  // _canCommand.qd_des_hip[0] = 0.0;
  // _canCommand.qd_des_hip[1] = 0.0;
  //
  //
  // _canCommand.kp_knee[0]= 15.0;
  // _canCommand.kp_knee[1]= 10.0;
  //
  // _canCommand.q_des_knee[0]= 0.50 + 0.5*fabs(sin(2*pi*omega*curr_time));
  // _canCommand.q_des_knee[1]= -0.25 + 0.5*fabs(sin(2*pi*omega*curr_time));
  //
  // _canCommand.qd_des_knee[0]= 0.0;
  // _canCommand.qd_des_knee[1]= 0.0;
  // _canCommand.q_des_abad[0]=-0.2;
  // _canCommand.kp_abad[0]=10.0;
  // _canCommand.kd_abad[0]=0.5;
  //
  // _canCommand.q_des_abad[1]= 0.2;
  // _canCommand.kp_abad[1]=10.0;
  // _canCommand.kd_abad[1]=0.5;


  #ifdef PRINT_CAN_DATA
  for(int leg=0; leg<2; leg++)
    printf("[CMD] Leg: %d A: %f H: %f K:%f \n", leg, _canCommand.q_des_abad[leg],
                                        _canCommand.q_des_hip[leg],
                                        _canCommand.q_des_knee[leg]
                                        );
  #endif
  memcpy(cmd, &_canCommand, sizeof(can_command_lcmt));
  can_write_run();

}
/*!
 * Run Pat CAN read
 */

void PatHardwareTestBridge::runCANRead() {
  can_data_lcmt *data = get_can_data();
  can_read_run();
  memcpy(&_canData, data, sizeof(can_data_lcmt));
  #ifdef PRINT_CAN_DATA
  for(int leg=0; leg<2; leg++){
    // if (_iter % 10 == 0)
       printf("[%d] Leg %d A: %f H: %f K: %f\n",
               (int)_iter_run_system,
               leg,
               _canData.q_abad[leg],
               _canData.q_hip[leg],
               _canData.q_knee[leg]);

   }
  #endif
}

void PatHardwareTestBridge::runSystem(){

  // if(_iter_run_system < 10){
  //   for(size_t i(0); i<pat_biped::num_legs; ++i) _spiCommand.flags[i] = 0;
  //
  // }else if(20 < _iter_run_system && _iter_run_system<30){
  //   for(size_t i(0); i<pat_biped::num_legs; ++i) _spiCommand.flags[i] = 0;
  //
  // }else if(40 < _iter_run_system && _iter_run_system<50){
  //   for(size_t i(0); i<pat_biped::num_legs; ++i) _spiCommand.flags[i] = 0;
  //
  // }else{
  //   for(size_t i(0); i<pat_biped::num_legs; ++i) _spiCommand.flags[i] = 1;

    // Estop
    if ( _system->Estop() ) {
      if (_iter_run_system % 1000 == 0)   printf("ESTOP!\n");
    }else{
      _system->onestep_forward();
    }
  // }
  ++_iter_run_system;

  //_system->updateFBModelStateWithEstimator();
  //_system->renderSystem();
}

void PatHardwareTestBridge::systemRendering(){
  _system->updateFBModelStateEstimate();
  _system->renderSystem();
}
/*!
 * Send LCM visualization data
 */
//void HardwareBridge::publishVisualizationLCM() {
  //cheetah_visualization_lcmt visualization_data;
  //for (int i = 0; i < 3; i++) {
    //visualization_data.x[i] = _mainCheetahVisualization.p[i];
  //}

  //for (int i = 0; i < 4; i++) {
    //visualization_data.quat[i] = _mainCheetahVisualization.quat[i];
    //visualization_data.rgba[i] = _mainCheetahVisualization.color[i];
  //}

  //for (int i = 0; i < 12; i++) {
    //visualization_data.q[i] = _mainCheetahVisualization.q[i];
  //}

  //_visualizationLCM.publish("main_cheetah_visualization", &visualization_data);
//}


#endif  // __linux
