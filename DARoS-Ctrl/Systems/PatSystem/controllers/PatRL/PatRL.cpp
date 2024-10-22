#ifdef MACHINE_LEARNING_BUILD

#include "PatRL.hpp"
#include "ParamHandler.hpp"
#include <Utilities/orientation_tools.h>
#include <iostream>
// #define PRINT_DEBUG

using namespace torch::indexing;

PatRL::PatRL(ControlFSMData<float>* data, PatParameters* parameters){

  _data = data;
  _parameters = parameters;
  _model = data->_pat->buildModel();

  _load_rl_config_params();
  _init_filters();
  _loadNormalizationFiles();
  _loadPolicy();


  _lcm = new lcm::LCM("udpm://239.255.76.67:7667?ttl=1");
  if (_lcm->good())
  {
     printf("LCM IN PatRL CONTROL INITIALIZED\n");
  }
  else
  {
     printf("LCM IN PatRL CONTROLLER FAILED\n");
     exit(-1);
  }
  std::cout << "[RL] Initializing LCM Interface...\n";
  std::cout << "[RL] Initializing LCM Interface...\n";


}
void PatRL::initialize() {
  _iter = 0;
  ESTOP = false;
  first_visit = true;
  _phases[1] = 0.0;
  _phases[0] = M_PI;
  _policy_input_vec.clear();
  _policy_input_vec.push_back(_policy_input_tensor);
  _historyLength = 6;
  _nJoints = pat_biped::num_act_joints;
  for(int i=0; i<6; i++){
    _default_dof_pos(i) = _default_dof_pos_vec[i];
  }
  for(int i=0; i<3; i++){
    _const_commands(i) = _const_commands_vec[i];
  }
  _dof_vel.setZero();
  _kpMat.setZero();
  _kdMat.setZero();
  _I_3.setIdentity();
  for(int i=0; i<2; i++) {
    _target_commands[i].zero();
  }
  _footPos.setZero(pat_biped::num_legs * 3);
  _obs.setZero(_obsDim);
  _obsMean.setZero(_obsDim);
  _obsVar.setZero(_obsDim);
  _previousAction.setZero(pat_biped::num_legs * 3);
  _prepreviousAction.setZero(pat_biped::num_legs * 3);
  _commands.setZero(3);

  _policy_cmd.resize(6);
  _policy_cmd.setZero();
  _jointPosErrorHist.setZero(_nJoints * _historyLength);
  _jointVelHist.setZero(_nJoints * _historyLength);
  _policy_cmd_filt.resize(6);_policy_cmd_filt.setZero();
  _policy_input_tensor = torch::zeros({1, _obsDim});
  _height_map.setZero();
  _begin = std::chrono::steady_clock::now();

}
void PatRL::run(){
  _end = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::microseconds>(_end - _begin).count() > int(_control_dt * 1000000)) {
    _begin = std::chrono::steady_clock::now();
  } else {
    for (int leg(0); leg < 2; ++leg) {
      _data->_legController->commands[leg] = _target_commands[leg];
    }
    _iter++;
    return;
  }
  #ifdef GAIT
  _updateGaitInfo();
    if(_use_ik_ref>0.0)
      for(size_t leg = 0; leg<pat_biped::num_legs; ++leg){
        _default_dof_pos.block<3,1>(3*leg, 0) = ik_ref_trajectory(_phases[RS_IDX(leg)], 0.1, RS_IDX(leg));
      }
  #endif
  // _updateObs();
  // _normalizeObs();
  // _runPolicy();
  // _updatePolicyCMD();

  _policy_cmd *= _action_scale;
  _policy_cmd.setZero();
  for(int i=0; i<6; i++){
    _policy_cmd(i) += _default_dof_pos(i);
  }

  double kneeGearRatio = 1.0; //14.49 / 9.;
  _kpMat = _I_3*getPGain()/(kneeGearRatio * kneeGearRatio);
  _kdMat = _I_3*getDGain()/(kneeGearRatio * kneeGearRatio);

  for (int leg(0); leg < 2; ++leg) {
    for (int jidx(0); jidx < 3; ++jidx) {
      _data->_legController->commands[RS_IDX(leg)].qDes[jidx] = _policy_cmd(leg * 3 + jidx);
      _data->_legController->commands[RS_IDX(leg)].qdDes[jidx] = 0.;
    }
    _data->_legController->commands[RS_IDX(leg)].kpJoint = _kpMat;
    _data->_legController->commands[RS_IDX(leg)].kdJoint = _kdMat;
    _target_commands[leg] = _data->_legController->commands[leg];
  }

  _iter++;

}
void PatRL::_loadPolicy(){

  _model_path = THIS_COM"config/" + _model_path;
  try {
    const char* m_path_cstr = _model_path.c_str();
    _torch_model = torch::jit::load(m_path_cstr);
    std::cout << "[RL] Successfully Loaded Policy Model..." << '\n';
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
  }
}
void PatRL::_init_filters(){
  for(int i=0; i<6; i++){
    _q_filt.push_back(digital_lp_filter<float>(2*M_PI*_filter_cutoff_freq, _filter_sample_dt));
  }
}
// void PatRL::_updateObs() {
//   auto res =_data->_stateEstimator->getResult();
//   Vec3<float> base_lin_vel_scaled, base_ang_vel_scaled, projected_gravity, commands_scaled;
//   Vec4<float> base_quat;
//   Vec6<float> dof_pos_diff, dof_vel_scaled, actions;
//
//   base_quat = res.orientation;
//   base_lin_vel_scaled = res.vBody*_obs_lin_vel_scale;
//   base_ang_vel_scaled = res.omegaBody*_obs_ang_vel_scale;
//   projected_gravity = quat_rotate_inverse(base_quat, _g_vec);
//
//
//   _bodyOri << flip_quat(res.orientation);
//   _bodyVel << res.getResult().vBody;
//   _bodyAngularVel << res.omegaBody;
//   _projected_gravity << quat_rotate_inverse(base_quat, _g_vec);
//   _jointQ << _data->_legController->datas[1].q, _data->_legController->datas[0].q;
//   _jointQd << _data->_legController->datas[1].qd, _data->_legController->datas[0].qd;
//   _footPos << _data->_pat->getHipLocation(1) +
//               _data->_legController->datas[1].p,
//               _data->_pat->getHipLocation(0) +
//               _data->_legController->datas[0].p,
//
//   if(_constant_cmd_enable>0.0){
//     commands_scaled[0] = _const_commands(0)*_obs_lin_vel_scale; //_data->_desiredStateCommand->data.stateDes(6)*_obs_lin_vel_scale;
//     commands_scaled[1] = _const_commands(1)*_obs_lin_vel_scale;
//     commands_scaled[2] = _const_commands(2)*_obs_ang_vel_scale;
//   }else{ //Keyboard input
//
//     commands_scaled[0] = _data->_desiredStateCommand->data.stateDes(6)*_obs_lin_vel_scale;
//     commands_scaled[1] = _data->_desiredStateCommand->data.stateDes(7)*_obs_lin_vel_scale;
//     commands_scaled[2] = _data->_desiredStateCommand->data.stateDes(11)*_obs_ang_vel_scale;
//
//   }
//   /*
//  */
//   for(int leg=0; leg<2; leg++){
//    for(int j=0; j<3; j++){
//        _dof_pos[3*leg + j] = _data->_legController->datas[RS_IDX(leg)]->q[j];
//        dof_pos_diff[3*leg + j] = _obs_dof_pos_scale*(_dof_pos[3*leg + j]-_default_dof_pos(3*RS_IDX(leg)+j));
//        _dof_vel[3*leg + j] = _data->_legController->datas[RS_IDX(leg)]->qd[j];
//        dof_vel_scaled[3*leg + j] = _obs_dof_vel_scale*_dof_vel[3*leg + j];
//      }
//   }
//   for(int i=0; i<6; i++){
//     actions[i] = _policy_cmd(i);
//   }
//
//   float body_height = _data->_stateEstimator->getResult().position[2];
//
//   _obs << body_height,//1
//           flip_quat(base_quat),//4
//           base_lin_vel_scaled,//3
//           base_ang_vel_scaled,//3
//           projected_gravity,//3
//           commands_scaled,//3
//           _dof_pos,//6
//           dof_vel_scaled,//6
//           actions//6
//           #ifdef ENABLE_GAIT
//           ,_phases[1]/(2*M_PI), //1 left leg phase
//            sin(_phases[1]), //1
//            cos(_phases[1]) //1
//           #endif
//           #ifdef ENABLE_PERCEPTION
//             , _height_map - 0.35 //187 Fake Height Map
//           #endif
//            ;
//
//   _policy_input_tensor = eigen_obs_to_torch(_obs);
//   _policy_input_vec.clear();
//   _policy_input_vec.push_back(_policy_input_tensor);
// }
void PatRL::_updateObs() {

  double contactThreshold = -0.4;
  Vec4<float> isContact; isContact.setZero();
  isContact << ((_policy_cmd(5) - _jointQ(5)) < contactThreshold),
      ((_policy_cmd(2) - _jointQ(2)) < contactThreshold),
      0, 0;
  _data->_stateEstimator->setContactPhase(isContact);
  _data->_stateEstimator->run();


  auto res =_data->_stateEstimator->getResult();
  _bodyOri << flip_quat(res.orientation);
  // _bodyVel << res.getResult().vBody;
  _bodyAngularVel << res.omegaBody;
  _projected_gravity << quat_rotate_inverse(res.orientation, _g_vec);
  _jointQ << _data->_legController->datas[1]->q, _data->_legController->datas[0]->q;
  _jointQd << _data->_legController->datas[1]->qd, _data->_legController->datas[0]->qd;
  _footPos << _data->_pat->getHipLocation(1) +
              _data->_legController->datas[1]->p,
              _data->_pat->getHipLocation(0) +
              _data->_legController->datas[0]->p;

  if(_constant_cmd_enable>0.0)
  {
    _commands[0] = _const_commands(0);
    _commands[1] = _const_commands(1);
    _commands[2] = _const_commands(2);
  }
  else{ //Keyboard input
    _commands[0] = _data->_desiredStateCommand->data.stateDes(6);
    _commands[1] = _data->_desiredStateCommand->data.stateDes(7);
    _commands[2] = _data->_desiredStateCommand->data.stateDes(11);
  }
  _obs << _bodyOri,//4
          _bodyAngularVel,//3
          _commands,//3
          _jointQ,//6
          _jointQd//6
          #ifdef JOINT_POS_ERROR_HIST
          ,_jointPosErrorHist.segment((_historyLength - 6) * _nJoints, _nJoints),//6
          _jointPosErrorHist.segment((_historyLength - 4) * _nJoints, _nJoints),//6
          _jointPosErrorHist.segment((_historyLength - 2) * _nJoints, _nJoints),//6
          #endif
          #ifdef JOINT_VEL_HIST
          _jointVelHist.segment((_historyLength - 6) * _nJoints, _nJoints), //6
          _jointVelHist.segment((_historyLength - 4) * _nJoints, _nJoints), //6
          _jointVelHist.segment((_historyLength - 2) * _nJoints, _nJoints) //6
          #endif
          ,_previousAction, //6
          _prepreviousAction //6
          #ifdef FOOT_POS
          , _footPos  /// 6 foot position with respect to the body COM, expressed in the body frame. 3 3 3 3
          #endif
          #ifdef GAIT
          ,_base_phase, //1 left leg phase
          sin(_phases[0]), //1
          sin(_phases[1]), //1
          cos(_phases[0]), //1
          cos(_phases[1]) //1
          #endif
          #ifdef PERCEPTION
          , _height_map - 0.35 //187 Fake Height Map
          #endif
          ;
  _policy_input_tensor = eigen_obs_to_torch(_obs);
  _policy_input_vec.clear();
  _policy_input_vec.push_back(_policy_input_tensor);
}
void PatRL::updatePreviousActions(){
  _prepreviousAction = _previousAction;
  _previousAction = _policy_cmd.cast<float>();
}
void PatRL::_runPolicy(){
  _policy_output = _torch_model.forward(_policy_input_vec).toTensor();

}
void PatRL::_updatePolicyCMD(){

  for(int i=0; i<6; i++){
    _policy_cmd(i) = _policy_output[0][i].item<double>();
    _q_filt[i].input((float)_policy_cmd(i));
    _policy_cmd_filt(i) = _q_filt[i].output();
    rl_output_publish.actions[i] = _policy_output[0][i].item<double>();
    rl_output_publish.actions_filtered[i] = _policy_cmd_filt(i);
  }

  _lcm->publish("CONTROLLER_rl_output", &rl_output_publish);
}
void PatRL::_load_rl_config_params(){

  ParamHandler handler(THIS_COM "config/pat-rl-parameters.yaml");
  handler.getVector<float>("default_joint_angles", _default_dof_pos_vec);
  handler.getValue<float>("stiffness", _P);
  handler.getValue<float>("damping", _D);
  handler.getValue<float>("action_scale", _action_scale);
  handler.getValue<float>("max_ori_error", _max_ori_error);
  handler.getValue<float>("max_height_error", _max_height_error);
  handler.getValue<float>("obs_lin_vel_scale", _obs_lin_vel_scale);
  handler.getValue<float>("obs_ang_vel_scale", _obs_ang_vel_scale);
  handler.getValue<float>("obs_dof_pos_scale", _obs_dof_pos_scale);
  handler.getValue<float>("obs_dof_vel_scale", _obs_dof_vel_scale);
  handler.getValue<float>("sim_dt", _sim_dt);
  handler.getValue<float>("control_dt", _control_dt);
  handler.getValue<float>("filter_cutoff_freq", _filter_cutoff_freq);
  handler.getValue<float>("filter_sample_dt", _filter_sample_dt);
  handler.getValue<float>("swing_time", _swing_time);
  handler.getValue<float>("decimation", _decimation);
  handler.getValue<float>("use_ik_ref", _use_ik_ref);
  handler.getValue<float>("constant_cmd_enable", _constant_cmd_enable);
  handler.getValue<float>("obs_dim", _obsDim_f);
  handler.getVector<float>("commands", _const_commands_vec);
  handler.getString("model_path", _model_path);
  printf("Done Parsing RL parameters\n");
  _obsDim = (int) _obsDim_f;
}
void PatRL::_loadNormalizationFiles(){
  std::string in_line;
  std::ifstream obsMean_file, obsVariance_file;
  obsMean_file.open(std::string(THIS_COM"config/") + "mean.csv");
  obsVariance_file.open(std::string(THIS_COM"config/") + "var.csv");

  if(obsMean_file.is_open()) {
    for(int i = 0; i < _obsMean.size(); i++){
      std::getline(obsMean_file, in_line);
      _obsMean(i) = std::stod(in_line);
    }
  }
  if(obsVariance_file.is_open()) {
    for(int i = 0; i < _obsVar.size(); i++){
      std::getline(obsVariance_file, in_line);
      _obsVar(i) = std::stod(in_line);
    }
  }
  obsMean_file.close();
  obsVariance_file.close();
}
void PatRL::_normalizeObs(){
  for (int i = 0; i < _obs.size(); i++) {
    _obs(i) = (_obs(i) - _obsMean(i)) / std::sqrt(_obsVar(i) + 1e-8);
    if (_obs(i) > 10) _obs(i) = 10.0;
    if (_obs(i) < -10) _obs(i) = -10.0;
  }
}
void PatRL::_updateHistory() {
  _historyTempMem = _jointVelHist;
  _jointVelHist.head((_historyLength-1) * _nJoints) = _historyTempMem.tail((_historyLength-1) * _nJoints);
  _jointVelHist.tail(_nJoints) = _jointQd;

  _historyTempMem = _jointPosErrorHist;
  _jointPosErrorHist.head((_historyLength-1) * _nJoints) = _historyTempMem.tail((_historyLength-1) * _nJoints);
  _jointPosErrorHist.tail(_nJoints) = _policy_cmd.cast<float>() - _jointQ;
}
Vec3<float> PatRL::ik_ref_trajectory(float phase, float swing_height, int leg){
  assert(phase>=0.0 && phase<=2*M_PI);
  /*
  Cubic Hermite Swing Trajectory
  */
  Vec3<float> des_foot_pos;
  Vec3<float> q;
  float t;
  float z_ref = 0.0;

  des_foot_pos<< 0.0, pow(-1, leg+1)*0.03, -0.35;
  if(phase<M_PI/2){ //Swing up
    t  = (2.0/M_PI)*phase;
    z_ref = swing_height*(-2*pow(t, 3) + 3*pow(t, 2));
  }
  else if(phase<M_PI) //Swing Down
  {
    t  = (2.0/M_PI)*phase - 1;
    z_ref = swing_height*(2*pow(t, 3) - 3*pow(t, 2) + 1);
  }
  else{ //Stance
  }
  des_foot_pos(2) += z_ref;
  q = analytical_IK(des_foot_pos);
  return q;
}
Vec3<float> PatRL::analytical_IK(Vec3<float> foot_pos){
  float a = 0.205; //knee link length
  float b = 0.2078; //hip link length
  float c = foot_pos.norm();
  float x = foot_pos(0);
  float y = foot_pos(1);
  float z = foot_pos(2);
  Vec3<float> q;
  q(0) = atan(y/(z+1e-8));
  q(1) = -(acos((b*b+c*c-a*a)/(2*b*c)) - atan(x/sqrt(y*y + z*z)));
  q(2) = M_PI - acos((a*a + b*b - c*c)/(2*a*b));
  return q;
}
void PatRL::_updateGaitInfo(){
  // _phases[0] = fmod(_phases[0] + _parameters->controller_dt, 2*_swing_time)/(2*_swing_time);
  //Right Leg swing First
  float _gait_period = 2*_swing_time;
  _base_phase = 2*M_PI*fmod(_iter*_parameters->controller_dt, _gait_period)/_gait_period;
  _delta_phases[0] = 0; // Left Leg swing First
  _delta_phases[1] = M_PI;
  _phases[0] = fmod(_base_phase + _delta_phases[0], 2*M_PI);
  _phases[1] = fmod(_base_phase + _delta_phases[1], 2*M_PI);
}
Vec3<float> PatRL::quat_rotate_inverse(Vec4<float>q, Vec3<float>v){

  Vec3<float> a, b, c, q_vec;
  float q_w = q(0);//real
  q_vec<<q(1), q(2), q(3);
  a = v * (2.0 * pow(q_w, 2) - 1.0);
  b = ori::crossMatrix(q_vec)*v*q_w*2.0;
  c = q_vec *(q_vec.transpose()*q_vec);
  return a - b + c;
}
Vec3<float> PatRL::quat_rotate(Vec4<float>q, Vec3<float>v){

  Vec3<float> a, b, c, q_vec;
  float q_w = q(3);
  q_vec = q.block<3,1>(0, 0);
  a = v * (2.0 * pow(q_w, 2) - 1.0);
  b = ori::crossMatrix(q_vec)*v*q_w*2.0;
  c = q_vec *(q_vec.transpose()*q_vec);
  return a + b + c;
}
Vec4<float> PatRL::flip_quat(const Vec4<float>q){

  Vec4<float> q_flip;
  q_flip(3) = q(0);
  q_flip(0) = q(1);
  q_flip(1) = q(2);
  q_flip(2) = q(3);
  return q_flip;
}
at::Tensor PatRL::eigen_obs_to_torch(Eigen::VectorXf obs_eig){
  at::Tensor obs_tensor = torch::zeros({1, _obsDim});
  for(int i=0; i< _obsDim; ++i){
    obs_tensor.index_put_({0, i}, obs_eig(i));
  }
  return obs_tensor;
}
void PatRL::print_obs(const at::Tensor obs){
  printf("Base Height");
  printf("\n\n");
  std::cout<< obs.index({0, 0})<<"\n";
  printf("\n\n");
  printf("Base Orientation");
  printf("\n\n");
  std::cout<< obs.index({0, Slice(1, 5)})<<"\n";
  printf("\n\n");
  printf("Base Lin vel");
  printf("\n\n");
  std::cout<< obs.index({0, Slice(5, 8)})<<"\n";
  printf("\n\n");
  printf("Base Ang vel");
  printf("\n\n");
  std::cout<< obs.index({0, Slice(8, 11)})<<"\n";
  printf("\n\n");
  printf("Proj. Grav\n");
  printf("\n\n");
  std::cout<< obs.index({0, Slice(11, 14)})<<"\n";
  printf("\n\n");
  printf("Commands\n");
  printf("\n\n");
  std::cout<< obs.index({0, Slice(14, 17)})<<"\n";
  printf("\n\n");
  printf("dof_pos\n");
  printf("\n\n");
  std::cout<< obs.index({0, Slice(17, 23)})<<"\n";
  printf("\n\n");
  printf("dof_vel\n");
  printf("\n\n");
  std::cout<< obs.index({0, Slice(23, 29)})<<"\n";
  printf("\n\n");
  printf("actions\n");
  printf("\n\n");
  std::cout<< obs.index({0, Slice(29, 35)})<<"\n";
  printf("\n\n");
  printf("phase\n");
  printf("\n\n");
  std::cout<< obs.index({0, 35})<<"\n";
  printf("\n\n");
  printf("sin phase\n");
  printf("\n\n");
  std::cout<< obs.index({0, 36})<<"\n";
  printf("\n\n");
  printf("cos phase\n");
  printf("\n\n");
  std::cout<< obs.index({0, 37})<<"\n";
  printf("\n\n");
}

#endif
