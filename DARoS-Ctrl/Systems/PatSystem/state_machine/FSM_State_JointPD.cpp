/*============================= Joint PD ==============================*/
/**
 * FSM State that allows PD control of the joints.
 */

#include "FSM_State_JointPD.h"
#include <Configuration.h>

/**
 * Constructor for the FSM State that passes in state specific info to
 * the generic FSM State constructor.
 *
 * @param _controlFSMData holds all of the relevant control data
 */
template <typename T>
FSM_State_JointPD<T>::FSM_State_JointPD(ControlFSMData<T>* _controlFSMData)
    : FSM_State<T>(_controlFSMData, FSM_StateName::JOINT_PD, "JOINT_PD"),
_ini_jpos(pat_biped::num_act_joints){
  // Do nothing here yet
}

template <typename T>
void FSM_State_JointPD<T>::onEnter() {
  // Reset counter
  iter = 0;

  for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
    for(size_t jidx(0); jidx <3; ++jidx){
      _ini_jpos[3*leg + jidx] = FSM_State<T>::_data->_legController->datas[leg]->q[jidx];
    }
  }

}

/**
 * Calls the functions to be executed on each control loop iteration.
 */
template <typename T>
void FSM_State_JointPD<T>::run() {
  // This is just a test, should be running whatever other code you want
  Vec3<T> qDes;
  // qDes << 0, -1., -2.0;
  qDes << 0.0, -1.0, 2.0;
  Vec3<T> qdDes;
  qdDes << 0, 0, 0;

  static double progress(0.);
  progress += this->_data->userParameters->controller_dt;
  double movement_duration(3.0);
  double ratio = progress/movement_duration;
  if(ratio > 1.) ratio = 1.;

  this->jointPDControl(0, ratio*qDes + (1. - ratio)*_ini_jpos.head(3), qdDes, 200, 2.0);
  this->jointPDControl(1, ratio*qDes + (1. - ratio)*_ini_jpos.segment(3, 3), qdDes, 200, 2.0);
  //_ModelChecking();
}

template <typename T>
void FSM_State_JointPD<T>::_ModelChecking(){
  StateEstimate<T> state_est = this->_data->_stateEstimator->getResult();
  LimbData<T>** leg_data =   this->_data->_legController->datas;
  FloatingBaseModel<T> _model = this->_data->_pat->buildModel();

  DMat<T> _A;
  DVec<T> _grav;

  FBModelState<T> _state;
  _state.q = DVec<T>::Zero(_model._nDof-6);
  _state.qd = DVec<T>::Zero(_model._nDof-6);


  _state.bodyOrientation = state_est.orientation;
  _state.bodyPosition = state_est.position;
  for(size_t i(0); i<3; ++i){
    _state.bodyVelocity[i] = state_est.omegaBody[i];
    _state.bodyVelocity[i+3] = state_est.vBody[i];

    for(size_t leg(0); leg<4; ++leg){
      _state.q[3*leg + i] = ((LegControllerPatData<T>*)leg_data[leg])->q[i];
      _state.qd[3*leg + i] = ((LegControllerPatData<T>*)leg_data[leg])->qd[i];
    }
  }
  _model.setState(_state);

  _model.contactJacobians();
  _model.massMatrix();//Comment if using massandCoriolisMatrix()
  _model.generalizedGravityForce();
  _model.generalizedCoriolisForce();
  _model.centroidMomentumMatrix();
  //_model.massandCoriolisMatrix();//Comment if not needed

  _A = _model.getMassMatrix();
  _grav = _model.getGravityForce();
  DVec<T> lin_grav = _grav.segment(3, 3);
  Mat3<T> rot = ori::quaternionToRotationMatrix(_model._state.bodyOrientation);
  lin_grav = rot.transpose() * lin_grav;

  pretty_print(_A, std::cout, "Mass");
  pretty_print(_grav, std::cout, "gravity");
  pretty_print(lin_grav, std::cout, "global lin gravity");
}
/**
 * Cleans up the state information on exiting the state.
 */
template <typename T>
void FSM_State_JointPD<T>::onExit() {
  // Nothing to clean up when exiting
}

//template class FSM_State_JointPD<double>;
template class FSM_State_JointPD<float>;
