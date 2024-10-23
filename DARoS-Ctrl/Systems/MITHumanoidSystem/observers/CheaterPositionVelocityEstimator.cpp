#include "CheaterPositionVelocityEstimator.h"
#include <pretty_print.h>

/*!
 * Run cheater estimator to copy cheater state into state estimate
 */
template <typename T>
void CheaterPositionVelocityEstimator<T>::run() {
  this->_stateEstimatorData->result.position = this->_stateEstimatorData->cheaterState->position.template cast<T>();
  this->_stateEstimatorData->result.vWorld =
    this->_stateEstimatorData->result.rBody.transpose().template cast<T>() * this->_stateEstimatorData->cheaterState->vBody.template cast<T>();
  this->_stateEstimatorData->result.vBody = this->_stateEstimatorData->cheaterState->vBody.template cast<T>();
}

template class CheaterPositionVelocityEstimator<float>;
template class CheaterPositionVelocityEstimator<double>;
