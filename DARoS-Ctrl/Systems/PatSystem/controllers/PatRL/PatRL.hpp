#ifndef VISIONRL_H
#define VISIONRL_H

#ifdef MACHINE_LEARNING_BUILD

#include <ControlFSMData.h>
#include "rl_inputs_lcmt.hpp"
#include "rl_outputs_lcmt.hpp"
#include <thread>
#include <iostream>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <Utilities/filters.h>

#include <torch/script.h>
#include <memory>

// #define ENABLE_PERCEPTION
#define HEIGHT_MAP_DIM 187
#define GAIT
// #define JOINT_POS_ERROR_HIST
// #define JOINT_VEL_HIST
#define FOOT_POS
// #define PERCEPTION
#define RS_IDX(i) ((i+1)%2) //Isaacgym index to Robot-Software index


class PatRL{
public:
	PatRL(ControlFSMData<float>* data, PatParameters* parameters);
	void initialize();
  void run();
  Vec6<float> getDefaultDofPos(){return _default_dof_pos;}
  Vec6<float> getDofDiffPos(){return _default_dof_pos-_dof_pos;}
  Vec6<float> getDofPos(){return _dof_pos;}
  Vec6<float> getDofVel(){return _dof_vel;}
	Eigen::VectorXd getPolicyCMD(){return _policy_cmd;}
	Eigen::VectorXd getPolicyCMDFiltered(){return _policy_cmd_filt;}
	float getPGain(){return _P;}
	float getDGain(){return _D;}
	float getActionScale(){return _action_scale;}
	float getMaxOriError(){return _max_ori_error;}
	float getMaxHeightError(){return _max_height_error;}
private:
  rl_outputs_lcmt rl_output_publish;
	Eigen::VectorXd _policy_cmd;
	Eigen::VectorXd _policy_cmd_filt;
	Vec6<float> _default_dof_pos;
	std::vector<float> _default_dof_pos_vec;
	Vec6<float> _dof_pos;
	Vec6<float> _dof_vel;
	Vec3<float> _g_vec = {0., 0., -1};

  std::vector<digital_lp_filter<float>> _q_filt;

  //config parameters
	float _P, _D, _action_scale;
	float _max_ori_error, _max_height_error;
  float _obs_lin_vel_scale, _obs_ang_vel_scale, _obs_dof_pos_scale, _obs_dof_vel_scale;
  float _sim_dt, _control_dt;
  float _filter_cutoff_freq, _filter_sample_dt;
  std::string  _model_path;


	lcm::LCM * _lcm;
  PatParameters* _parameters = nullptr;
	ControlFSMData<float>* _data = nullptr;
  FloatingBaseModel<float> _model;
  FBModelState<float> _state;
  std::chrono::steady_clock::time_point _begin;
  std::chrono::steady_clock::time_point _end;
  //pytorch stuff
	// const char* _model_path;
	torch::jit::script::Module _torch_model;
	at::Tensor _policy_output, _policy_input_tensor;
	std::vector<torch::jit::IValue> _policy_input_vec;
  Eigen::VectorXf _obs, _obsMean, _obsVar;
  Eigen::Matrix<float, HEIGHT_MAP_DIM, 1> _height_map;
  Mat3<float> _kpMat, _kdMat, _I_3;
  float _nJoints, _historyLength;
  int _obsDim; float _obsDim_f;

  Eigen::Matrix<float, 4, 1> _bodyOri;
  Eigen::Matrix<float, 3, 1> _bodyVel;
  Eigen::Matrix<float, 3, 1> _commands;
  Eigen::Matrix<float, 3, 1> _bodyAngularVel;
  Eigen::Matrix<float, 6, 1> _jointQ;
  Eigen::Matrix<float, 6, 1> _jointQd;
  Eigen::VectorXf _jointPosErrorHist, _jointVelHist, _historyTempMem;
  Eigen::VectorXf _previousAction, _prepreviousAction;
  Eigen::VectorXf _footPos;


  Eigen::Matrix<float, 6, 1> _jointQDes;

  Eigen::Matrix<float, 3, 1> _projected_gravity;


  void _init_filters();
  void _load_rl_config_params();
	void _runPolicy();
  void _updateObs();
  void _updateHistory();
  void _updatePolicyCMD();
  void _updateGaitInfo();
  void _loadNormalizationFiles();
  void _normalizeObs();
  void _loadPolicy();
  void updatePreviousActions();

  // Count Iterations
	int _iter = 0;
  float _phases[2];
  float _delta_phases[2];
  float _base_phase;
  float _swing_time;
  float _decimation;
  float _use_ik_ref;
  float _constant_cmd_enable;
  Vec3<float> _const_commands;
  std::vector<float> _const_commands_vec;
  LegControllerPatCommand<float> _target_commands[2];
	// Account for initial yaw
	bool first_visit = true;
  bool ESTOP = false;

  //helper functions
  Vec3<float> analytical_IK(Vec3<float> foot_pos);
  Vec3<float> ik_ref_trajectory(float phase, float swing_height, int leg);
	Vec3<float> quat_rotate_inverse(Vec4<float>q, Vec3<float>v);
	Vec3<float> quat_rotate(Vec4<float>q, Vec3<float>v);
  Vec4<float> flip_quat(const Vec4<float>q);
  at::Tensor eigen_obs_to_torch(Eigen::VectorXf obs_eig);
  void print_obs(const at::Tensor obs);
};
#endif

#endif
