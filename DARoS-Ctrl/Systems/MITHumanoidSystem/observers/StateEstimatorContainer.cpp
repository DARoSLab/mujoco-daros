#include "StateEstimatorContainer.h"
#include <Utilities/pretty_print.h>

template<typename T>
StateEstimatorContainer<T>::StateEstimatorContainer(CheaterState* cheaterState,
    IMU_Data* imuData,
    JointControlData<T>** jointData,
    EstimatorParameters* parameters,
    Robot<T>* robot) {

  _data.cheaterState = cheaterState;
  _data.imuData = imuData;
  _data.jointData = jointData;
  _data.contactPhase.setZero();
  _data.footHeights.setZero();
  _data.parameters = parameters;
  _data.robot = robot;
}

template <typename T>
void StateEstimatorContainer<T>::run(){
  for (auto estimator : _estimators) {
    estimator->run();
  }
}


template class StateEstimatorContainer<double>;
template class StateEstimatorContainer<float>;
