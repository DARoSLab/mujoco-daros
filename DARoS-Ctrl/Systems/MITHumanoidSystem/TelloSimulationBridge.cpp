#include "TelloSimulationBridge.hpp"
#include "Tello_State_Parkour.h"
TelloSimulationBridge::TelloSimulationBridge(System<double>* sys,
    const std::string sim_params_file, const std::string & terrain_file):
SimulationBridge(sys, sim_params_file, terrain_file){

    
  
  //simon's testing code
  // double mu=0.7, resti=0, depth=1, width=1, height=1, transparent=0;
  // double pos[3] = {1, 0, -0.45};
  // double ori[3] = {0, 0, 0};
  // Mat3<double> R_box = ori::rpyToRotMat(Vec3<double>(ori));
  // R_box.transposeInPlace();
  // _sim->addCollisionBox(mu, resti, depth, width, height, Vec3<double>(pos), R_box,
  //         true, false);

  // Simulated sensors (each system has different sensors)
  // init IMU simulator
  printf("[Tello Simulation] Setup IMU simulator...\n");
  _imuSimulator = new IMU_Simulator<double>(_sim_params);
  _tello_sys = dynamic_cast<TelloSystem<double> * >(_system);
  _tello_sys->_sys_data = &_j_data;
  _tello_sys->_sys_cmd = &_j_cmd;
  _tello_sys->_sys_imuData = &_imuData;
  _tello_sys->_sys_cheaterState = &_cheaterState;
  //_tello_sys->_sys_gamepadCommand = &_gamepadCommand;

  _tello_sys->initialization();

  _ctrl_dt = _tello_sys->getParameters().controller_dt;
  _sim_dt = _sim_params.dynamics_dt;

  if(_ctrl_dt < _sim_dt){
    printf("[Error] Controller runs faster than Simulation\n");
    exit(0);
  }
  //run simulation before run controller
  _ctrl_time += _ctrl_dt;

  printf("[Tello Simulation Bridge] Constructed\n");
  //access terrain
  Tello_State_Parkour <float> * parkour_state = dynamic_cast<Tello_State_Parkour<float> *>(_tello_sys->_controlFSM->_state_list[5]);
  Eigen::VectorXd terrain = parkour_state->_casadiMPC->terrain;
  // adding collision box
  double mu=0.7, resti=0, depth=0.1, width=1, height=1, transparent=0;
  double ori[3] = {0, 0, 0};
  Mat3<double> R_box = ori::rpyToRotMat(Vec3<double>(ori));
  R_box.transposeInPlace();
  for (int i = 0; i < terrain.size(); i++){
    if (terrain(i) > 0){
      double pos[3] = {i*0.05+0.025, 0, terrain(i)-width/2.0};
      _sim->addCollisionBox(mu, resti, depth, width, height, Vec3<double>(pos), R_box,
              true, false);
    }
  }
}

void TelloSimulationBridge::_onestep_simulation(){

  // Create force arrows for LCM message to Unity visualization
  //int gc_point[4] = {0, 1, 2, 3}; 
  //for (int i = 0; i < 4; i++){
    //auto forceArrow = _tello_sys->_visualizationData->addArrow();
    //forceArrow->head_width = 0.0125;
    //forceArrow->head_length = 0.04;
    //forceArrow->shaft_width = 0.004;
    //forceArrow->color={1.0, 0.0, 0.0, 0.7};
    //for (int xyz = 0; xyz < 3; xyz++){
      //size_t gcID = _sim_dyn_model._footIndicesGC.at(gc_point[i]);
      //forceArrow->base_position[xyz] = _sim_dyn_model._pGC.at(gcID)[xyz];
      //forceArrow->direction[xyz] = _sim->getDynamicsEngine()->getContactForce(gcID)[xyz];
    //}
  //}
  
  // update actuator command
  for(size_t i(0); i<tello::num_act_joint; ++i){
    _actuator_models[i]->updateCMD(_j_cmd.q_des[i], _j_cmd.qd_des[i], 
        _j_cmd.tau_ff[i], _j_cmd.kp_joint[i], _j_cmd.kd_joint[i]);
  }

  while(_sim_time < _ctrl_time){
    _sim->step(_actuator_models, _state, _dstate);
    _sim_time += _sim_dt;
  }

  _imuSimulator->updateIMU(_state, _dstate, &_imuData);
  _imuSimulator->updateCheaterState(_state, _dstate, _cheaterState);

  for(size_t i(0); i<tello::num_act_joint; ++i){
    _actuator_models[i]->updateCMD(_j_cmd.q_des[i], _j_cmd.qd_des[i], 
        _j_cmd.tau_ff[i], _j_cmd.kp_joint[i], _j_cmd.kd_joint[i]);
  }
  for(size_t i(0); i < tello::num_act_joint; ++i){
    _j_data.q[i] = _state.q[i]; 
    _j_data.qd[i] = _state.qd[i]; 
  }
}

void TelloSimulationBridge::_onestep_system(){
  _system->setFBModelState(_state);
  _system->updateFBModelStateEstimate();
  _system->onestep_forward();
  _ctrl_time += _ctrl_dt;
}
