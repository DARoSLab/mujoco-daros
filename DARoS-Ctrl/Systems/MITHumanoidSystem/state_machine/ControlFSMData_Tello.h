#ifndef CONTROLFSMDATA_Tello_H
#define CONTROLFSMDATA_Tello_H

#include <TelloParameters.h>
#include <controllers/auxillary/TelloJointController.h>
#include <controllers/auxillary/UserInputHandler.h>
#include <observers/StateEstimatorContainer.h>
#include <robots/Tello.h>
#include <SimUtilities/VisualizationData.h>


/**
 *
 */
template <typename T>
struct ControlFSMData_Tello {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FloatingBaseModel<T>* _tello_model = NULL;
  StateEstimatorContainer<T>* _stateEstimator;
  TelloJointController<T>* _jointController;
  TelloParameters* _userParameters;
  VisualizationData* _visualizationData;
  UserInputHandler<T>* _userInputHandler;
};

template struct ControlFSMData_Tello<double>;
template struct ControlFSMData_Tello<float>;

#endif  // CONTROLFSM_H
