/*============================= Recovery Stand ==============================*/
/**
 * Transitionary state that is called for the robot to stand up into
 * balance control mode.
 */

#include "FSM_State_RecoveryStand.h"
#include <Utilities/pretty_print.h>
#include <ParamHandler/ParamHandler.hpp>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_RecoveryStand<T>::FSM_State_RecoveryStand(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::RECOVERY_STAND, "RECOVERY_STAND"){
  // Do nothing
  // Set the pre controls safety checks
  this->checkSafeOrientation = false;

  // Post control safety checks
  this->checkPDesFoot = false;
  this->checkForceFeedForward = false;
  _model = this->_data->_pat->buildModel();
  _state.q = DVec<T>::Zero(_model._nDof-6);
  _state.qd = DVec<T>::Zero(_model._nDof-6);
  zero_vec3.setZero();
  // goal configuration
  // Folding knee bent backwards
  // fold_jpos[0] << 0.3f, 0.55f, -0.95f;
  // fold_jpos[1] << -0.16f, 0.55f, -0.95f;

  // fold_jpos[0] << 0.16f, -0.3f, 0.83f;
  // fold_jpos[1] << -0.3f, -0.29f, 0.81f;

  //old
  // fold_jpos[0] << 0.16f, -0.46f, 1.2f;
  // fold_jpos[1] << -0.3f, -0.46f, 1.2f;
  //testing 0.40
  // fold_jpos[0] << 0.16f, -0.46f, 1.2f;
  // fold_jpos[1] << -0.16f, -0.46f, 1.2f;

  //testing 0.42
  // fold_jpos[0] << 0.16f, -0.45f, 0.75f;
  // fold_jpos[1] << -0.32f, -0.45f, 0.75f;
  _ParameterSetup(THIS_COM"config/pat-recovery-parameters.yaml");
  // Stand Up
  stand_jpos[0] << fold_jpos[0];
  stand_jpos[1] << fold_jpos[1];
  // stand_jpos[0] << 0.0f, -0.0f, 0.0f;
  // stand_jpos[1] << 0.0f, -0.0f, 0.0f;


  f_ff << 0.f, 0.f, -25.f;
}

template <typename T>
void FSM_State_RecoveryStand<T>::onEnter() {
  // Reset iteration counter
  iter = 0;
  _state_iter = 0;
  b_standup_settled = false;

  // initial configuration, position
  for(size_t i(0); i < 2; ++i) {
    initial_jpos[i] = this->_data->_legController->datas[i]->q;
  }


  //Determine which side to roll based on falling orientation (roll from state estimator)

  T body_height = this->_data->_stateEstimator->getResult().position[2];

  _flag = FoldLegs;
  // if( !_UpsideDown() ) { // Proper orientation
  //   if (  (0.2 < body_height) && (body_height < 0.45) ){
  //     printf("[Recovery Stand] body height is %f; Stand Up \n", body_height);
  //     _flag = StandUp;
  //   }else{
  //     printf("[Recovery Stand] body height is %f; Folding legs \n", body_height);
  //   }
  // }else{
  //     printf("[Recovery Stand] UpsideDown (%d) \n", _UpsideDown() );
  // }
  _motion_start_iter = 0;
  printf("[Recovery Stand] OnEnter\n");
}
template<typename T>
void FSM_State_RecoveryStand<T>::_UpdateModel(){
  const StateEstimate<T> & state_est = this->_data->_stateEstimator->getResult();
  LimbData<T> * const * leg_data = this->_data->_legController->datas;

  _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = state_est.position;
  for(size_t i(0); i<3; ++i){
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i+3] = state_est.vBody[i];
    for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
      _state.q[3*leg + i] = ((LegControllerPatData<T>*)leg_data[leg])->q[i];
      _state.qd[3*leg + i] = ((LegControllerPatData<T>*)leg_data[leg])->qd[i];
    }
  }
  _model.setState(_state);
  _model.contactJacobians();
}

template <typename T>
bool FSM_State_RecoveryStand<T>::_UpsideDown(){
  //if(this->_data->_stateEstimator->getResult().aBody[2] < 0){
  if(this->_data->_stateEstimator->getResult().rBody(2,2) < 0){
    return true;
  }
  return false;
}


/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_RecoveryStand<T>::run() {

  // std::cout << "com vel: " << this->_data->mocapData->com_vel << '\n';
  // std::cout << "com pos: " << this->_data->mocapData->com_pos << '\n';
  // std::cout << "marker orientation: " << this->_data->mocapData->marker_orientation << '\n';
  // //_UpdateModel();
  for(int foot=0; foot<2; foot++){
    // size_t gcID = _model._footIndicesGC.at(foot);
    // auto foot_position = _model._pGC.at(gcID);
    // std::cout <<"Foot: " << foot <<" Height: " << foot_position << '\n';
  }
  _model.forwardKinematics();
  // auto body_position = _model._pGC.at(5);
  // std::cout << "Body position FB " << body_position << '\n';
  // std::cout << "Body position EST " << _state.bodyPosition << '\n';

  switch(_flag){
    case StandUp:
      _StandUp(_state_iter - _motion_start_iter);
      break;
    case FoldLegs:
      _FoldLegs(_state_iter - _motion_start_iter);
      break;
  }
  Vec4<T> conphase;conphase.setZero();
  // if(b_standup_settled)
  conphase<<0.5, 0.5, 0.5, 0.5;
  this->_data->_stateEstimator->setContactPhase(conphase);
  ++_state_iter;
}

template <typename T>
void FSM_State_RecoveryStand<T>::_SetJPosInterPts(
    const size_t & curr_iter, size_t max_iter, int leg,
    const Vec3<T> & ini, const Vec3<T> & fin){

    float a(0.f);
    float b(1.f);

    // if we're done interpolating
    if(curr_iter <= max_iter) {
      b = (float)curr_iter/(float)max_iter;
      a = 1.f - b;
    }

    // compute setpoints
    Vec3<T> inter_pos = a * ini + b * fin;

    // do control
    this->jointPDControl(leg, inter_pos, zero_vec3, 100.0, 3);
    // if(curr_iter%100==0)
    //   std::cout << "OMEGA world: " << this->_data->_stateEstimator->getResult().omegaWorld << '\n';
}


template <typename T>
void FSM_State_RecoveryStand<T>::_StandUp(const int & curr_iter){
  T body_height = this->_data->_stateEstimator->getResult().position[2];
  bool something_wrong(false);

  // if( _UpsideDown() || (body_height < 0.1 ) ) {
  //   something_wrong = true;
  // }

  // if( (curr_iter > floor(standup_ramp_iter*0.7) ) && something_wrong){
  //   // If body height is too low because of some reason
  //   // even after the stand up motion is almost over
  //   // (Can happen when E-Stop is engaged in the middle of Other state)
  //   for(size_t i(0); i < 4; ++i) {
  //     initial_jpos[i] = this->_data->_legController->datas[i]->q;
  //   }
  //   _flag = FoldLegs;
  //   _motion_start_iter = _state_iter+1;
  //
  //   //Update which side to roll based on falling orientation
  //   if( this->_data->_stateEstimator->getResult().rpy(0) >=0 ){ //roll left
  //     roll_left=true;
  //   }else{ //roll right
  //     roll_left=false;
  //   }
  //   _AssignRollDirection();
  //
  //   printf("[Recovery Stand - Warning] body height is still too low (%f) or UpsideDown (%d); Folding legs \n",
  //       body_height, _UpsideDown() );
  //
  // }else{
    for(size_t leg(0); leg<2; ++leg){
      _SetJPosInterPts(curr_iter, standup_ramp_iter,
          leg, initial_jpos[leg], stand_jpos[leg]);
    }
    if(curr_iter > standup_ramp_iter + standup_settle_iter){
      b_standup_settled = true;
    // }
  }
  // feed forward mass of robot.
  //for(int i = 0; i < 4; i++)
  //this->_data->_legController->commands[i].forceFeedForward = f_ff;
  //Vec4<T> se_contactState(0.,0.,0.,0.);
  Vec4<T> se_contactState(0.5,0.5,0.5,0.5);
  Vec4<T> foot_height; foot_height.setZero();
  this->_data->_stateEstimator->setContactPhase(se_contactState);
  this->_data->_stateEstimator->setFootHeights(foot_height);

}

template <typename T>
void FSM_State_RecoveryStand<T>::_FoldLegs(const int & curr_iter){

  for(size_t i(0); i<2; ++i){
    _SetJPosInterPts(curr_iter, fold_ramp_iter, i,
        initial_jpos[i], fold_jpos[i]);
  }
  if(curr_iter >= fold_ramp_iter + fold_settle_iter){
    // if(_UpsideDown()){
    //   _flag = RollOver;
    //   for(size_t i(0); i<4; ++i) initial_jpos[i] = fold_jpos[i];
    // }else{
      _flag = StandUp;
      for(size_t i(0); i<2; ++i) initial_jpos[i] = fold_jpos[i];
    // }
    _motion_start_iter = _state_iter + 1;
  }
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_RecoveryStand<T>::onExit() {
  // Nothing to clean up when exiting
}

template<typename T>
void FSM_State_RecoveryStand<T>::_ParameterSetup(const std::string & file){
  ParamHandler handler(file);



  handler.getVector("ll_stand_jpos", _ll_stand_jpos);
  handler.getVector("rl_stand_jpos", _rl_stand_jpos);


   for(int i(0); i<3; i++){
     fold_jpos[0][i] = _rl_stand_jpos[i];
     fold_jpos[1][i] = _ll_stand_jpos[i];
   }
   std::cout << "fold_jpos 0 : " << fold_jpos[0] << '\n';
   std::cout << "fold_jpos 1: " << fold_jpos[1] << '\n';
  printf("[RECOVERY_STAND] Parameter Setup is completed\n");


}
// template class FSM_State_RecoveryStand<double>;
template class FSM_State_RecoveryStand<float>;
