/*=========================== Balance Stand ===========================*/
/**
 * FSM State that forces all legs to be on the ground and uses the QP
 * Balance controller for instantaneous balance control.
 */

#include "FSM_State_IK_Stepping.h"
#include <wbc_ctrl/LocomotionCtrl/LocomotionCtrl.hpp>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_IK_Stepping<T>::FSM_State_IK_Stepping(
    ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::BALANCE_STAND,"BALANCE_STAND"), _lcm(getLcmUrl(255)){
  // Set the pre controls safety checks
  this->turnOnAllSafetyChecks();
  // Turn off Foot pos command since it is set in WBC as operational task
  this->checkPDesFoot = false;
  this->footFeedForwardForces = Mat32<T>::Zero();
  // Initialize GRF to 0s
  ;
  _model = this->_data->_pat->buildModel();
  _state.q = DVec<T>::Zero(_model._nDof-6);
  _state.qd = DVec<T>::Zero(_model._nDof-6);
}

template <typename T>
void FSM_State_IK_Stepping<T>::onEnter() {
  _iter = 0;
  _phase[0] = 0.0;
  _phase[1] = M_PI;
}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_IK_Stepping<T>::run() {
  _UpdateModel();
  BalanceStandStep();
}

/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_IK_Stepping<T>::onExit() {
  _iter = 0;
  _phase[0] = 0.0;
  _phase[1] = M_PI;
}
template<typename T>
void FSM_State_IK_Stepping<T>::_UpdateModel(){

  const StateEstimate<T> & state_est = this->_data->_stateEstimator->getResult();
  LimbData<T> * const * leg_data = this->_data->_legController->datas;
  Vec3<T> upside_down_ori;
  upside_down_ori << 0.0, M_PI, 0.0;

  _state.bodyOrientation = ori::rpyToQuat(upside_down_ori);
  _state.bodyPosition.setZero();
  _state.bodyVelocity.setZero();

  for(size_t i(0); i<3; ++i){
    for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
      _state.q[3*leg + i] = ((LegControllerPatData<T>*)leg_data[leg])->q[i];
      _state.qd[3*leg + i] = ((LegControllerPatData<T>*)leg_data[leg])->qd[i];

    }
  }
  _model.setState(_state);
  _model.forwardKinematics();
  int linkid =  state_est.contactEstimate(0) > 0.0 ? pat_biped_linkID::RF : pat_biped_linkID::LF;
  _state.bodyPosition = -_model._pGC[linkid];
  _state.bodyVelocity.tail(3) = -_model._vGC[linkid];

  _model.setState(_state);

  for(int i(0); i<3; i++){
    _ik_lcm.rf_pos[i] = _model._pGC[pat_biped_linkID::RF][i];
    _ik_lcm.lf_pos[i] = _model._pGC[pat_biped_linkID::LF][i];
  }

}

/**
 * Calculate the commands for the leg controllers for each of the feet.
 */
template <typename T>
void FSM_State_IK_Stepping<T>::BalanceStandStep() {
  UpdateGaitInfo();
  _iter++;
  Vec3<T> zero_vec3; zero_vec3.setZero();
  for(size_t leg = 0; leg<pat_biped::num_legs; ++leg){
    auto qdes = SwingTrajectory(_phase[leg], 0.05, leg);
    for(int i(0); i<3; i++){
      _ik_lcm.q[3*leg + i] = this->_data->_legController->datas[leg]->q[i];
      _ik_lcm.qdes[3*leg + i] = qdes[i];
      _ik_lcm.error[3*leg + i] = abs(qdes[i]-_ik_lcm.q[3*leg + i]);

    }

    this->jointPDControl(leg, qdes, zero_vec3, 100, 3);
  }
  _lcm.publish("ik_stepping_tracking_data", &_ik_lcm);
}
template <typename T>
Vec3<T> FSM_State_IK_Stepping<T>::SwingTrajectory(T phase, T swing_height, int leg){
  assert(phase>=0.0 && phase<=2*M_PI);
  /*
  Cubic Hermite Swing Trajectory
  */
  T z_ref = 0.0;
  T t;
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
  Vec3<T> des_foot_pos;
  // if(leg == 1)
  des_foot_pos<< 0.0, pow(-1, leg+1)*0.06, -0.35 + z_ref;
  // else
  //   des_foot_pos<< 0.0, pow(-1, leg+1)*0.06, -0.35;

  Vec3<T> q = Analytical_IK(des_foot_pos);
  return q;
}
template <typename T>
Vec3<T> FSM_State_IK_Stepping<T>::Analytical_IK(Vec3<T> foot_pos){
  T a = 0.1955;
  T b = 0.205;
  T c = foot_pos.norm();
  T x = foot_pos(0);
  T y = foot_pos(1);
  T z = foot_pos(2);
  Vec3<T> q;
  q(0) = atan(y/(z+1e-8));
  q(1) = -(acos((b*b+c*c-a*a)/(2*b*c)) - atan(x/sqrt(y*y + z*z)));
  q(2) = M_PI - acos((a*a + b*b - c*c)/(2*a*b));
  return q;
}

template <typename T>
void FSM_State_IK_Stepping<T>::UpdateGaitInfo(){
  _phase[0] = 2*M_PI*(fmod(_iter*0.002, _gait_period)/_gait_period);//left leg
  _phase[0] = fmod(_phase[0], 2*M_PI);
  _phase[1] = fmod(_phase[0]+ M_PI, 2*M_PI);//right leg
  Vec4<T> conphase;conphase.setZero();
  T r = 1.0;
  for(size_t foot(0); foot < pat_biped::num_legs; foot++){
    if(_phase[foot]>r*M_PI)
        conphase[foot] = (_phase[foot]-r*M_PI)/((2-r)*M_PI);

    // conphase[foot] = _phase[foot]>M_PI? (_phase[foot]-M_PI)/ M_PI : 0.0;
    // conphase[foot] = _phase[foot]>M_PI? 0.5 : 0;
  }

  // std::cout << "IK contact: " << conphase << '\n';
  this->_data->_stateEstimator->setContactPhase(conphase);
}
// template class FSM_State_IK_Stepping<double>;
template class FSM_State_IK_Stepping<float>;
