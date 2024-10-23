#include "PatSystem.hpp"
#include <PatLCMCommunicator.hpp>
#include <PatMocapManager.hpp>
#include <pretty_print.h>
#include <observers/PositionVelocityEstimator.h>
#include <observers/PatKinematicMocapEstimator.h>
#include <observers/PatEstimators.h>
#include <Utilities/Timer.h>
#include <state_machine/ControlFSMData.h>
template <typename T>
PatSystem<T>::PatSystem(const char* robot_type, bool is_sim):System<T>(is_sim)
{
  this->_robot = new PatBiped<T>();
  _pat_ctrl = new PatBiped<float>();

  PatBiped<T> * pat = dynamic_cast<PatBiped<T>* > (this->_robot);
  // if(robot_type[0] == 'p'){
  _robot_type = RobotType::PATROCLUS;
  if(std::isdigit(robot_type[0]))
    _robot_id = atoi(robot_type);
  std::cout << "Pat sys: robot id " << _robot_id<< '\n';
  buildPatroclus(pat);
  // buildPatroclus(_pat_ctrl);
  buildPatroclusControl(_pat_ctrl);
  printf("*** Patroclus ***\n");

  // }else{
  //   throw std::runtime_error("not valid robot type selection");
  // }
  this->_robot->getInitialState(this->_state);
  this->_robot->getInitialState(_estimated_state);

  printf("setting system parameters from a yaml file\n");
  _sys_parameters.initializeFromYamlFile(THIS_COM"config/pat-parameters.yaml");
  _sys_estimator_parameters.initializeFromYamlFile(THIS_COM"config/pat-estimator-parameters.yaml");

  _legController = new LegControllerPat<float>(*_pat_ctrl);
  this->_update_dt = _sys_parameters.controller_dt;
  _gaitScheduler = new GaitSchedulerPat<float>(&_sys_parameters, this->_update_dt, _pat_ctrl);

  _visualizationData = new VisualizationData();
  _mocapData = new MoCapData<float>();
  _sys_gamepadCommand = new GamepadCommand();
  _jpos_initializer = new JPosInitializerPat<float>(3., this->_update_dt);

  if(this->_is_sim){
    _communicator = new PatLCMCommunicator<T>(this, 255);
  }else{
    _communicator = new PatLCMCommunicator<T>(this, 255);
 }
  // _mocap_manager = new PatMocapManager<T>(this);

  if(!is_sim){ _simulation_test = 0; }
  else{ _simulation_test = _sys_parameters.simulation_test; }
}

template <typename T>
bool PatSystem<T>::initialization(){
// Must be initialized (construct estimator, command, control fsm) after
// _sys_canData, _sys_canCommand,  _sys_vectorNavData,  _sys_localizationData,
// _sys_cheaterState, _sys_gamepadCommand
// are allocated
  if(_initialized){
    printf("[Pat System - Warning] Multiple Initialization Attempts\n");
    return _initialized;
  }else{
    // _stateEstimator = new StateEstimatorContainer<float>(
    //     _sys_cheaterState, _sys_vectorNavData, _sys_localizationData,
    //     &_legController->datas[0],
    //     &_sys_estimator_parameters,
    //     _pat_ctrl);
    _stateEstimator = new StateEstimatorContainer<float>(
        _sys_cheaterState, _sys_vectorNavData, _sys_mocapData, _sys_localizationData,
        &_legController->datas[0],
        &_sys_estimator_parameters,
        _pat_ctrl);

printf("i-1\n");
    memset(&rc_control, 0, sizeof(rc_control_settings));

    _desiredStateCommand = new DesiredStateCommand<float>(_sys_gamepadCommand,
        &rc_control,
        _stateEstimator,
        &_sys_parameters.PATH_move_waypoint,
        &_sys_parameters.use_rc,
        _sys_parameters.controller_dt); // &user

printf("i-1\n");
    _controlFSM = new ControlFSM<float>(_pat_ctrl,
        _stateEstimator,
        _legController, _gaitScheduler, _desiredStateCommand,
        &_sys_parameters, _visualizationData, _mocapData, _robot_id);

printf("i-2\n");
    _initializeStateEstimator(false);

printf("i-3\n");
    _legController->zeroCommand();
    _legController->setEnabled(true);

    get_xsr_control_settings(&rc_control);
printf("i-4\n");

    _initialized = true;
    return _initialized;
  }
}

template<typename T>
void PatSystem<T>::onestep_forward(){
  //Timer tic;
  get_xsr_control_settings(&rc_control);
  // if(!_cheaterModeEnabled && this->_is_sim){
  //
  //   printf("[RobotRunner] Transitioning to Cheater Mode...\n");
  //   _initializeStateEstimator(true);
  //   _cheaterModeEnabled = true;
  // }
  // if(_cheaterModeEnabled && !this->_is_sim){
  //
  //   printf("[RobotRunner] Transitioning from Cheater Mode...\n");
  //   _initializeStateEstimator(false);
  //   _cheaterModeEnabled = false;
  // }
  // check transition to cheater mode:
  if (!_cheaterModeEnabled && _sys_parameters.cheater_mode) {
    printf("[RobotRunner] Transitioning to Cheater Mode...\n");
    _initializeStateEstimator(true);
    //todo any configuration
    _cheaterModeEnabled = true;
  }
  // check transition from cheater mode:
  if (_cheaterModeEnabled && !_sys_parameters.cheater_mode) {
    printf("[RobotRunner] Transitioning from Cheater Mode...\n");
    _initializeStateEstimator(false);
    // todo any configuration
    _cheaterModeEnabled = false;
  }
  _legController->updateModelOriAndVel(_stateEstimator->getResult().orientation,
                                        _stateEstimator->getResult().omegaBody);
  // Update Jpos, Jvel
  _legController->updateData(_sys_canData);

  // Clear previous leg controller commands
  _legController->zeroCommand();
  _legController->setEnabled(true);

  // Run State Estmiator
  _stateEstimator->run();


  // Find the current gait schedule
  _gaitScheduler->step();

  // if( _simulation_test || _jpos_initializer->IsInitialized(_legController)){
  // if( _simulation_test || _jpos_initializer->IsInitialized(_legController)){

    // Find the desired state trajectory
  _desiredStateCommand->convertToStateCommands();

    // FSM State Controller
  _controlFSM->runFSM();
  // }

  _legController->updateCommand(_sys_canCommand);
  _legController->setLcm(&leg_control_data_lcm, &leg_control_command_lcm);
  _stateEstimator->setLcm(&state_estimator_lcm);

  // if(this->_is_sim){
    _communicator->finalizeStep(&leg_control_data_lcm,
        &leg_control_command_lcm,
        &state_estimator_lcm);
  // }
  //_ave_time += tic.getMs()/(T)num_skip;

  //if(iter%num_skip == 0){
    //printf("average one step forward (ms): %f\n", _ave_time);
    //_ave_time = 0.;
  //}
  ++iter;
}

template<typename T>
void PatSystem<T>::_initializeStateEstimator(bool cheaterMode) {
  _stateEstimator->removeAllEstimators();
  _stateEstimator->addEstimator<ContactEstimator<float>>();

  Vec4<float> contactDefault;
  contactDefault << 0.5, 0.5, 0.5, 0.5; // last two are dummy
  _stateEstimator->setContactPhase(contactDefault);
  if (cheaterMode) {
    _stateEstimator->addEstimator<PatCheaterOrientationEstimator<float>>();
    _stateEstimator->addEstimator<CheaterPositionVelocityEstimator<float>>();

  } else {
    _stateEstimator->addEstimator<PatVectorNavOrientationEstimator<float>>();
    _stateEstimator->addEstimator<LinearKFPositionVelocityEstimator<float>>();
    if(!this->_is_sim){
      _stateEstimator->addEstimator<PatKinematicMocapEstimator<float>>();
    }
    // _stateEstimator->addEstimator<MoCapEstimator<float>>();
  }
}

template<typename T>
bool PatSystem<T>::Estop(){
  get_xsr_control_settings(&rc_control);
  //static int count(0);
  //++count;
  //if(count %300 ==0){
    //printf("rc mode: %f\n", rc_control.mode);
  //}
  if ( (rc_control.mode == RC_mode::OFF) && _sys_parameters.use_rc ) {
    _controlFSM->initialize();

    // Update Jpos, Jvel
    _legController->updateData(_sys_canData);

    // Run State Estmiator
    _stateEstimator->run();

    // zero command
    for (int leg = 0; leg < this->_robot->NUM_FEET; leg++) {
      _legController->commands[leg].zero();
    }
    _legController->updateCommand(_sys_canCommand);
    return true;
  }
  return false;
}

template<typename T>
void PatSystem<T>::updateFBModelStateEstimate(){
  _estimated_state.bodyOrientation = _stateEstimator->getResult().orientation.template cast<T>();
  _estimated_state.bodyPosition = _stateEstimator->getResult().position.template cast<T>();

  for(int leg(0); leg<this->_robot->NUM_FEET; ++leg){
    for(int jidx(0); jidx<3; ++jidx){
      _estimated_state.q[3*leg + jidx] = _legController->datas[leg]->q[jidx];
    }
  }
}

template class PatSystem<double>;
template class PatSystem<float>;
