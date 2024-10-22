#include "PatSimulationBridge.hpp"
#include <Configuration.h>
#include <Utilities/orientation_tools.h>
#include <ParamHandler/ParamHandler.hpp>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <Configuration.h>

PatSimulationBridge::PatSimulationBridge(System<double> *sys,
                                         const std::string &sim_params_file, const std::string &terrain_file) : SimulationBridge(sys, sim_params_file, terrain_file)
{
  // Simulated sensors (each system has different sensors)
  // init IMU simulator
  printf("[RS_Simulation] Setup IMU simulator...\n");
  _imuSimulator = new ImuSimulator<double>(_sim_params);

  // init Real sense localization simulator
  printf("[RS_Simulation] Setup RS Localization simulator...\n");
  _rsSimulator = new realSenseSimulator<double>(_sim_params);

  _pat_sys = dynamic_cast<PatSystem<double> *>(_system);

  _pat_sys->_sys_canData = &_canData;
  _pat_sys->_sys_canCommand = &_spiCommand;
  _pat_sys->_sys_vectorNavData = &_vectorNavData;
  _pat_sys->_sys_cheaterState = &_cheaterState;
  _pat_sys->_sys_localizationData = &_localizationData;
  _pat_sys->_sys_gamepadCommand = &_gamepadCommand;

  _pat_sys->initialization();

  _ctrl_dt = _pat_sys->getParameters().controller_dt;
  _sim_dt = _sim_params.dynamics_dt;
  _enable_support = _pat_sys->getParameters().use_rc;
  if (_ctrl_dt < _sim_dt)
  {
    printf("[Error] Controller runs faster than Simulation\n");
    exit(0);
  }
  _ParamExtractor(sim_params_file);
  //run simulation before run controller
  _ctrl_time += _ctrl_dt;
  //XSR-SIM DONGLE RC controller Task
  // init_xsr();
  // PeriodicMemberFunction<PatSimulationBridge> xsrTask(
  //       &taskManager, .001, "rc_controller", &PatSimulationBridge::runXSR, this);// 0.005
  //   xsrTask.start();
  //


}
// void PatSimulationBridge::runXSR(){
//   run_xsr();
//   xsr_complete();
// }
void PatSimulationBridge::_ParamExtractor(const std::string &file_name)
{
  ParamHandler params(THIS_COM "config/simulation-experiment.yaml");
  params.getVector("enable_external_force", _enable_ext_force);
  params.getVector("Kp", _ext_force_kp);
  params.getVector("Kd", _ext_force_kd);
  params.getValue("max_disable_count", _max_disable_count);
  params.getValue("enable_fixture_support", _enable_fixture_support);
  params.getValue("enable_push_test", _enable_push_test);
  params.getVector("des_body_pose", _des_body_pose);
  std::cout<<"Sim bridge Robot ID: "<<_pat_sys->getRobotId() << "\n";
  ParamHandler ext_params(THIS_COM "config/external_force_params/" + std::to_string(_pat_sys->getRobotId()) + ".yaml");
  ext_params.getVector("external_force_time", _ext_force_time);
  ext_params.getVector("external_force_duration", _ext_force_duration);
  // ext_params.get2DArray("external_force_list", _ext_force_list);
  ext_params.getVector("external_force", _ext_force);




  _num_impulse = _ext_force_time.size();
  _force_idx = 0;
  if (_num_impulse > 0)
    _b_no_more_force = false;

  //printf("num: %zu,\n ", _num_impulse);
  //for(size_t i(0); i<_num_impulse; ++i){
  //printf("%zu th: %f, %f\n, ", i, _ext_force_time[i], _ext_force_duration[i]);
  //printf("%f, %f, %f\n, ", _ext_force_list[i][0], _ext_force_list[i][1], _ext_force_list[i][2]);
  //}
}

void PatSimulationBridge::_onestep_simulation()
{
  Vec3<double> rf_sum;
  rf_sum.setZero();
  // Create force arrows for LCM message to Unity visualization
  if (this->_sim_dyn_model._footIndicesGC.size() > 0)
  {
    for (size_t cp = 0; cp < pat_biped::num_legs; cp++)
    {
      auto forceArrow = _pat_sys->_visualizationData->addArrow();
      forceArrow->head_width = 0.0125;
      forceArrow->head_length = 0.04;
      forceArrow->shaft_width = 0.004;
      forceArrow->color = {1.0, 0.0, 0.0, 0.7};
      // T forceArrowScale = 300.f;
      for (int i = 0; i < 3; i++)
      {
        size_t gcID = this->_sim_dyn_model._footIndicesGC.at(cp);
        // _debug_lcmt.f_foot_contact[cp][i] = this->_sim->getDynamicsEngine()->getContactForce(gcID)[i];
        // _debug_lcmt.p_foot_contact[cp][i] = this->_sim_dyn_model._pGC.at(gcID)[i];
        forceArrow->base_position[i] = this->_sim_dyn_model._pGC.at(gcID)[i];
        forceArrow->direction[i] = this->_sim->getDynamicsEngine()->getContactForce(gcID)[i];

        rf_sum[i] += forceArrow->direction[i];
      }
      // std::cout << "[sim] Foot " << cp <<" position "<< forceArrow->base_position<< '\n';
      //printf("%lu th force: %f, %f, %f\n", cp,
      //forceArrow->direction[0],
      //forceArrow->direction[1],
      //forceArrow->direction[2]);
    }
  }

  // body bottom reaction force drawing
  //for(size_t idx(4); idx < this->_sim_dyn_model._pGC.size(); ++idx){
  //auto forceArrow =_pat_sys->_visualizationData->addArrow();
  //forceArrow->head_width = 0.0125;
  //forceArrow->head_length = 0.04;
  //forceArrow->shaft_width = 0.004;
  //forceArrow->color={1.0, 0.0, 0.0, 0.7};
  //for (int i = 0; i < 3; i++){
  //forceArrow->base_position[i]=this->_sim_dyn_model._pGC.at(idx)[i];
  //forceArrow->direction[i]=this->_sim->getDynamicsEngine()->getContactForce(idx)[i];
  //}
  //printf("%lu th force: %f, %f, %f\n", idx,
  //forceArrow->direction[0],
  //forceArrow->direction[1],
  //forceArrow->direction[2]);
  //}

  //pretty_print(rf_sum, std::cout, "Sim bridge, rf sum");

  // update actuator command
  for (size_t i(0); i < pat_biped::num_legs; ++i)
  {
    _actuator_models[3 * i]->updateCMD(
        _spiCommand.q_des_abad[i],
        _spiCommand.qd_des_abad[i],
        _spiCommand.tau_abad_ff[i],
        _spiCommand.kp_abad[i], _spiCommand.kd_abad[i]);

    _actuator_models[3 * i + 1]->updateCMD(
        _spiCommand.q_des_hip[i],
        _spiCommand.qd_des_hip[i],
        _spiCommand.tau_hip_ff[i],
        _spiCommand.kp_hip[i], _spiCommand.kd_hip[i]);

    _actuator_models[3 * i + 2]->updateCMD(
        _spiCommand.q_des_knee[i],
        _spiCommand.qd_des_knee[i],
        _spiCommand.tau_knee_ff[i],
        _spiCommand.kp_knee[i], _spiCommand.kd_knee[i]);
  }
  while (_sim_time < _ctrl_time)
  {

    auto forceArrow = _pat_sys->_visualizationData->addArrow();
    forceArrow->head_width = 0.0125;
    forceArrow->head_length = 0.1;
    forceArrow->shaft_width = 0.004;
    forceArrow->color = {0.0, 1.0, 0.0, 0.7};
    if(_enable_push_test){
      // External Force Setting
      if ((!_b_no_more_force) &&
          _ext_force_time[_force_idx] < _sim_time &&
          _sim_time < _ext_force_time[_force_idx] + _ext_force_duration[_force_idx])
      {

        Vec3<double> f;
        // f << _ext_force_list[_force_idx][0],
        //     _ext_force_list[_force_idx][1],
        //     _ext_force_list[_force_idx][2];
        f << _ext_force[0],
            _ext_force[1],
            _ext_force[2];


        Vec3<double> p;
        p.setZero();
        int link_id = 5; //body

        for (int i = 0; i < 3; i++)
        {
          forceArrow->base_position[i] = this->_sim_dyn_model.getPosition(5)[i];
          forceArrow->direction[i] = -5*f[i];
        }
        _sim->setExternalForce(forceToSpatialForce(f, p), link_id);
        // printf("sim time: %f, external force: %f, %f, %f\n", _sim_time, f[0], f[1], f[2]);
      }
      else
      {
        for (int i = 0; i < 3; i++)
        {
          forceArrow->base_position[i] = this->_sim_dyn_model.getPosition(5)[i];
          forceArrow->direction[i] = 0.0;
        }
        if (_sim_time > _ext_force_time[_force_idx] + _ext_force_duration[_force_idx])
        {
          ++_force_idx;
        }
        if (_force_idx == _num_impulse)
          _b_no_more_force = true;
      }
    }
    int link_id = 5;
    Vec3<double> f; f.setZero();
    Vec6<double> ft; ft.setZero();
    Vec3<double> p; p.setZero();




    Vec3<double> p_body = this->_sim_dyn_model.getPosition(link_id);
    Vec3<double> v_body = this->_sim_dyn_model.getLinearVelocity(link_id);
    Vec3<double> omega_body = this->_sim_dyn_model.getAngularVelocity(link_id);
    Mat3<double> R_body = this->_sim_dyn_model.getOrientation(link_id);
    Vec3<double> error_R_lin; error_R_lin.setZero();
    Vec6<double> p_error, v_error; p_error.setZero(); v_error.setZero();
    Mat3<double> R_ini = ori::coordinateRotation(CoordinateAxis::Z, _des_body_pose[2]);
    auto R = -0.5*(R_ini.transpose()*R_body-R_body.transpose()*R_ini);
    error_R_lin(0) = R(2,1);
    error_R_lin(1) = R(0,2);
    error_R_lin(2) = R(1,0);
    p_error << error_R_lin, _des_body_pose[3]-p_body(0), _des_body_pose[4]-p_body(1), _des_body_pose[5]-p_body(2);
    v_error << -omega_body, -v_body;
    // std::cout << "enable support: " << _enable_support << '\n';
    // std::cout << "enable fixture support: " << _enable_fixture_support << '\n';
    // std::cout << "_max_disable_count: " << _max_disable_count << '\n';
    if(!_enable_disable_counter &&
        _enable_support &&
        _pat_sys->getParameters().use_rc<1.0){
          _enable_disable_counter = true;
          // std::cout << "enabling disable counter" << '\n';
    }
    if(_enable_disable_counter && _enable_support){

      if(++_wait_disable_counter > _max_disable_count){
        _enable_support = false;
        _enable_disable_counter = false;
      }else{

        // printf("Waiting to disable  %d \n", _wait_disable_counter);
      }
    }
    if(_enable_support)
      for(int i(0); i<6; ++i){
        if(_enable_ext_force[i]>0.0)
            ft(i) = _ext_force_kp[i]*p_error(i) + _ext_force_kd[i]*v_error(i);
      }
    else
      ft.setZero();

    if(_enable_fixture_support)
      _sim->setExternalForce(ft, link_id);
    // _sim->setExternalForce(forceToSpatialForce(f, p), link_id);

    _sim->step(_actuator_models, _state, _dstate);
    _sim_time += _sim_dt;

    //printf("ctrl time, sim time: %f, %f\n", _ctrl_time, _sim_time);
  }
  _imuSimulator->updateVectornav(_state, _dstate, &_vectorNavData, true);
  _imuSimulator->updateCheaterState(_state, _dstate, _cheaterState);
  _rsSimulator->updateLocalization(_state, _dstate, &_localizationData);

  for(size_t i(0); i<2; ++i){
    _canData.q_abad[i] = _state.q[3*i];
    _canData.q_hip[i] = _state.q[3*i + 1];
    _canData.q_knee[i] = _state.q[3*i + 2];

    _canData.qd_abad[i] = _state.qd[3*i];
    _canData.qd_hip[i] = _state.qd[3*i + 1];
    _canData.qd_knee[i] = _state.qd[3*i + 2];
  }
}

void PatSimulationBridge::_onestep_system()
{
  _system->setFBModelState(_state);
  _system->updateFBModelStateEstimate();
  _system->onestep_forward();
  _ctrl_time += _ctrl_dt;
}
