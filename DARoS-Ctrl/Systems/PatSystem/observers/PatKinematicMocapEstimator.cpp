/*! @file PatKinematicMocapEstimator.cpp
 *  @brief All State Estimation Algorithms
 *
 *  This file will contain all state estimation algorithms.
 *  PatKinematicMocapEstimators should compute:
 *  - body position/velocity in world/body frames
 *  - foot positions/velocities in body/world frame
 */

#include "PatKinematicMocapEstimator.h"
#include <pretty_print.h>
#include <robots/Patroclus.h>
template <typename T>
PatKinematicMocapEstimator<T>::PatKinematicMocapEstimator():_lcm(getLcmUrl(255)) {
  _pat = new PatBiped<T>();
  buildPatroclus(_pat);
  _model = _pat->buildModel();
  _state.q = DVec<T>::Zero(_model._nDof-6);
  _state.qd = DVec<T>::Zero(_model._nDof-6);
}

template <typename T>
void PatKinematicMocapEstimator<T>::setup() {



}
template<typename T>
void PatKinematicMocapEstimator<T>::_UpdateModel(){

  // _state.bodyOrientation << this->_stateEstimatorData->>result.orientation;
  // _state.bodyVelocity.head(3)<< this->_stateEstimatorData->>result.omegaBody;
  Vec3<T> upside_down; upside_down << 0.0, 0.0, 0.0;
  _state.bodyOrientation << ori::rpyToQuat(upside_down);
  _state.bodyPosition.setZero();
  _state.bodyVelocity.head(3).setZero();
  _state.bodyVelocity.tail(3).setZero();

  for(size_t i(0); i<3; ++i){
    for(size_t leg(0); leg<pat_biped::num_legs; ++leg){
      _state.q[3*leg + i] = this->_stateEstimatorData->>limbData[leg]->q[i];
      _state.qd[3*leg + i] = this->_stateEstimatorData->>limbData[leg]->qd[i];
    }
  }
  _model.setState(_state);
  _model.forwardKinematics();
  _stance_foot_id =  this->_stateEstimatorData->>result.contactEstimate(0) > 0.0 ? pat_biped_linkID::RF : pat_biped_linkID::LF;
  _state.bodyPosition = -_model._pGC[_stance_foot_id];
  _state.bodyVelocity.tail(3) = -_model._vGC[_stance_foot_id];
  _model.setState(_state);
  _model.forwardKinematics();

}


/*!
 * Run state estimator
 */
template <typename T>
void PatKinematicMocapEstimator<T>::run() {
  _UpdateModel();
  Mat3<T> w_R_m; w_R_m << 1, 0, 0,
                          0, 1, 0,
                          0,  0, 1;
  auto rf_pos_kin = _model._pGC[pat_biped_linkID::RF]-_model._state.bodyPosition; //relative base frame in world coordinate
  auto lf_pos_kin = _model._pGC[pat_biped_linkID::LF]-_model._state.bodyPosition;

  auto local_body_pos = this->_stateEstimatorData->>mocapData->local_poses[0];
  auto local_lf_pos = this->_stateEstimatorData->>mocapData->local_poses[6];
  auto local_rf_pos = this->_stateEstimatorData->>mocapData->local_poses[7];
  auto rf_pos_mocap_raw = local_rf_pos - local_body_pos;
  auto lf_pos_mocap_raw = local_lf_pos - local_body_pos;

  Vec3<T> rf_pos_mocap = w_R_m*rf_pos_mocap_raw ;
  Vec3<T> lf_pos_mocap = w_R_m*lf_pos_mocap_raw ;
  Quat<T> quat; quat << _model._state.bodyOrientation;
  Mat3<T> R = quaternionToRotationMatrix(quat).transpose();
  Vec3<T> body_offset; body_offset<<-0.05, 0, 0;
  rf_pos_mocap = rf_pos_mocap - R*body_offset;
  lf_pos_mocap = lf_pos_mocap - R*body_offset;
  // lf_pos_mocap += R*body_offset;

  for(int i(0); i<3; i++){
    _kin_mocap_lcm.rf_pos_kin[i] = rf_pos_kin[i];
    _kin_mocap_lcm.rf_pos_mocap[i] = rf_pos_mocap[i];
    _kin_mocap_lcm.lf_pos_kin[i] = lf_pos_kin[i];
    _kin_mocap_lcm.lf_pos_mocap[i] = lf_pos_mocap[i];
    _kin_mocap_lcm.lf_pos_mocap_raw[i] = lf_pos_mocap_raw[i];
    _kin_mocap_lcm.rf_pos_mocap_raw[i] = rf_pos_mocap_raw[i];
    _kin_mocap_lcm.rf_pos_in_stance[i] = _model._pGC[pat_biped_linkID::RF][i];
    _kin_mocap_lcm.lf_pos_in_stance[i] = _model._pGC[pat_biped_linkID::LF][i];
    _kin_mocap_lcm.body_pos_kin[i] = _model._state.bodyPosition[i];
    _kin_mocap_lcm.body_pos_mocap[i] = local_body_pos[i];

  }
  _kin_mocap_lcm.stance_foot_index = _stance_foot_id==10? 0 : 1;
  _lcm.publish("pat_mocap_kin_lcmt", &_kin_mocap_lcm);

}


template class PatKinematicMocapEstimator<float>;
template class PatKinematicMocapEstimator<double>;


/*!
 * Run cheater estimator to copy cheater state into state estimate
 */
template <typename T>
void CheaterPatKinematicMocapEstimator<T>::run() {
  this->_stateEstimatorData->>result.position = this->_stateEstimatorData->>cheaterState->position.template cast<T>();
  this->_stateEstimatorData->>result.vWorld =
    this->_stateEstimatorData->>result.rBody.transpose().template cast<T>() * this->_stateEstimatorData->>cheaterState->vBody.template cast<T>();
  this->_stateEstimatorData->>result.vBody = this->_stateEstimatorData->>cheaterState->vBody.template cast<T>();
}

template class CheaterPatKinematicMocapEstimator<float>;
template class CheaterPatKinematicMocapEstimator<double>;
