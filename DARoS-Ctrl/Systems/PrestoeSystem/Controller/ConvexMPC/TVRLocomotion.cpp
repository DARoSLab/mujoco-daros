#include "TVRLocomotion.h"
#include <iostream>
#include <utilities/Timer.h>
#include <utilities/pretty_print.h>
#include <ParamHandler/ParamHandler.hpp>

#define ORIGINAL_IMPLEMENTATION
// #define DRAW_DEBUG_SWINGS
////////////////////
// Controller
////////////////////

TVRLocomotion::TVRLocomotion(PatParameters* parameters,
    const FloatingBaseModel<float> * model):
  _model(model),
  _tvrLCM(getLcmUrl(255))
{
  _parameters = parameters;
  _LoadLocomotionParams(THIS_COM"config/pat-locomotion-parameters.yaml");
  _planner = new Reversal_LIPM_Planner();
  _planner->PlannerInitialization(THIS_COM"config/pat-locomotion-parameters.yaml");
  _planner->setOmega(_body_height);//body height

  _sgait = new SimpleGait(_swing_time, _alpha, _beta, _parameters->controller_dt);

  _swing_offset[0] = _alpha*_swing_time;
  _swing_offset[1] = (1 + 2*_alpha)*_swing_time;
  _sgait->setGaitPeriod((2 + 2*_alpha)*_swing_time);
  _sgait->setSwingOffest(_swing_offset);

  for(size_t i = 0; i < 3; i++){
      _min_jerk_offset.push_back(new MinJerk_OneDimension());
  }

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();

}

void TVRLocomotion::initialize(){
  for(int i = 0; i < _num_cp; i++){
    _previous_cs[i] = 1; // starts in double stance
  }
  _enable_swing_planning = false;
  _step_count = -1;
  _iter = 0;
  _sgait->resetPhase();
}




void TVRLocomotion::run(ControlFSMData<float>& data) {
  // Command Setup
  _setupCommand(data);
  //update gait information
  _sgait->updateGait();
  _swing_states = _sgait->getSwingState();
  _contact_states = _sgait->getContactState();


  //update COM and Foot Position
  _com_pos = _model->_state.bodyPosition + _model->getComPosWorld(); // world frame
  _com_vel = _model->getComVel();

  for(int k(0); k<2; k++){
    _tvr_lcm.COM_pos[k] = _com_pos[k];
    _tvr_lcm.COM_vel[k] = _com_vel[k];
  }

  _pFoot[0] = _model->_pGC[pat_biped_linkID::RF];
  _pFoot[1] = _model->_pGC[pat_biped_linkID::LF];

  for(int leg(0); leg<2; leg++){
      _current_cs[leg] = _contact_states[leg]>0? 1 : 0;
  }
  if(_contact_states[0] > 0.0 && _contact_states[1] > 0.0){//double stance
    _enable_swing_planning = true;
    _step_count++;
  }else{
    _planSwingLegTrajectory();

  }
  for(int leg(0); leg<2; leg++){
     _previous_cs[leg] =  _current_cs[leg];
  }
  // Update For WBC
  _updateWBCCMD();

  //update state-estimator contact state
  Vec4<float> se_contactState; se_contactState << _contact_states, 0, 0;
  data._stateEstimator->setContactPhase(se_contactState);
  _tvr_lcm.step_count = _step_count;
  _tvrLCM.publish("tvr_data_lcmt", &_tvr_lcm);
  _iter++;

}
void TVRLocomotion::_setupCommand(ControlFSMData<float> & data){

  _x_vel_des = 0.0;
  _y_vel_des = 0.0;
  _x_com_des += 0.5*_x_vel_des*_parameters->controller_dt;
  _y_com_des += 0.5*_y_vel_des*_parameters->controller_dt;


}
void TVRLocomotion::_runTVRPlanning(){

  OutputReversalPL pl_output;
  ParamReversalPL pl_param;
  Vec2<float> des_com_location;
  Vec3<float> target_offset;


  des_com_location<< _x_com_des, _y_com_des;
  _com_pos_ini << _com_pos;
  _com_vel_ini << _com_vel;
  _stance_foot_loc_ini << _pFoot[_stance_leg];

  pl_param.swing_time = 0.5*_swing_time;
  pl_param.des_loc = des_com_location;
  pl_param.stance_foot_loc = _pFoot[_stance_leg];
  pl_param.b_positive_sidestep = _swing_states[1]>0.0;

  _planner->getNextFootLocation(_com_pos,
                                _com_vel,
                                _target_foot_loc,
                                &pl_param,
                                &pl_output);


  _target_foot_loc[2] = _default_foot_loc[2];
  target_offset = _target_foot_loc - _init_target_foot_loc;
  target_offset.setZero();
  _SetMinJerkOffset(target_offset);


}
void TVRLocomotion::_planSwingLegTrajectory(){

  _swing_leg = _swing_states[0]>0.0? 0 : 1;
  _stance_leg = (_swing_leg + 1) % 2;
  if(_current_cs[_swing_leg]<_previous_cs[_swing_leg]){//beginning of swing


    _init_target_foot_loc  = _model->_state.bodyPosition;
    _init_target_foot_loc [1] += pow(-1, _swing_leg+1)*_default_foot_loc[1];
    _init_target_foot_loc [2] = _default_foot_loc[2];



    _SetBspline(_pFoot[_swing_leg], _init_target_foot_loc);
    _SetBazier(_pFoot[_swing_leg], _init_target_foot_loc);

  }else{

    if(_swing_states[_swing_leg]>_tvr_plan_time && _enable_swing_planning){

      _enable_swing_planning = false;
      _runTVRPlanning();

    }


  }
  _GetBsplineSwingTrajectory();
  double traj_time = (_swing_states[_swing_leg] - _tvr_plan_time)*_swing_time;
  if(_swing_states[_swing_leg] > _tvr_plan_time){
    double pos, vel, acc;
    for(int i(0); i<3; ++i){
        _min_jerk_offset[i]->getPos(traj_time, pos);
        _min_jerk_offset[i]->getVel(traj_time, vel);
        _min_jerk_offset[i]->getAcc(traj_time, acc);

        pFoot_des[_swing_leg][i] += pos;
        vFoot_des[_swing_leg][i] += vel;
        aFoot_des[_swing_leg][i] += acc;
    }
  }
  // _GetBazierSwingTrajectory();

}
void TVRLocomotion::_updateWBCCMD(){

  pBody_des << _model->_state.bodyPosition;
  pBody_des[2] = _body_height;

  vBody_des.setZero();
  aBody_des.setZero();

  pBody_RPY_des[0] = 0.;
  pBody_RPY_des[1] = 0.;
  pBody_RPY_des[2] = _yaw_des;

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;

  contact_state = _sgait->getContactState();
  fsm_state = _sgait->getFSMState();
  fsm_phase = _sgait->getFSMPhase();
}

void TVRLocomotion::_GetBsplineSwingTrajectory(){
    double pos[3];
    double vel[3];
    double acc[3];
    _foot_traj.getCurvePoint(_swing_states[_swing_leg]*_swing_time, pos);
    _foot_traj.getCurveDerPoint(_swing_states[_swing_leg]*_swing_time, 1, vel);
    _foot_traj.getCurveDerPoint(_swing_states[_swing_leg]*_swing_time, 2, acc);

    for(int i(0); i<3; ++i){
        pFoot_des[_swing_leg][i] = pos[i];
        vFoot_des[_swing_leg][i] = vel[i];
        aFoot_des[_swing_leg][i] = acc[i];
    }
}

void TVRLocomotion::_SetMinJerkOffset(const Vec3<float> & offset){
    // Initialize Minimum Jerk Parameter Containers
    Vec3<float> init_params;
    Vec3<float> final_params;

    // Set Minimum Jerk Boundary Conditions
    for(size_t i = 0; i < 3; i++){
        // Set Dimension i's initial pos, vel and acceleration
        init_params.setZero();
        // Set Dimension i's final pos, vel, acceleration
        final_params.setZero();
        final_params[0] = offset[i];

        _min_jerk_offset[i]->setParams(
                init_params, final_params,
                0., (1-_tvr_plan_time)*_swing_time);
    }
}
void TVRLocomotion::_SetBspline(
            const Vec3<float> & st_pos,
            const Vec3<float> & des_pos){
    // Trajectory Setup
    double init[9];
    double fin[9];
    double** middle_pt = new double*[1];
    middle_pt[0] = new double[3];
    Vec3<float> middle_pos;

    middle_pos = (st_pos + des_pos)/2.;
    middle_pos[2] = _swing_height;

    // Initial and final position & velocity & acceleration
    for(int i(0); i<3; ++i){
        // Initial
        init[i] = st_pos[i];
        init[i+3] = 0.;
        init[i+6] = 0.;
        // Final
        fin[i] = des_pos[i];
        fin[i+3] = 0.;
        fin[i+6] = 0.;
        // Middle
        middle_pt[0][i] = middle_pos[i];
    }
    // TEST
    fin[5] = -0.5;
    fin[8] = 5.;
    _foot_traj.SetParam(init, fin, middle_pt, _swing_time);

    delete [] *middle_pt;
    delete [] middle_pt;
}

void TVRLocomotion::_SetBazier(const Vec3<float> & st_pos, const Vec3<float> & des_pos){
  footSwingTrajectory.setInitialPosition(st_pos);
  footSwingTrajectory.setFinalPosition(des_pos);
  footSwingTrajectory.setHeight(_swing_height);
}


void TVRLocomotion::_GetBazierSwingTrajectory(){

  footSwingTrajectory.computeSwingTrajectoryBezier(_swing_states[_swing_leg], _swing_time);
  pFoot_des[_swing_leg] = footSwingTrajectory.getPosition();
  vFoot_des[_swing_leg] = footSwingTrajectory.getVelocity();
  aFoot_des[_swing_leg] = footSwingTrajectory.getAcceleration();

}

void TVRLocomotion::_LoadLocomotionParams(const std::string & file){
  ParamHandler handler(file);

  handler.getValue("swing_time", _swing_time);
  handler.getValue("swing_height", _swing_height);
  handler.getValue("body_height", _body_height);
  handler.getValue("alpha", _alpha);
  handler.getValue("beta", _beta);
  handler.getValue("tvr_plan_time", _tvr_plan_time);
  handler.getVector("default_foot_location", _default_foot_loc);
  printf("[WBC_biped Planner] Parameter Setup is completed\n");

}
