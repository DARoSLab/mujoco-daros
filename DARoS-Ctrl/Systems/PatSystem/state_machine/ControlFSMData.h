#ifndef PAT_CONTROLFSMDATA_H
#define PAT_CONTROLFSMDATA_H

#include <PatParameters.h>
#include <control/ctrl_utils/DesiredStateCommand.h>
#include <auxillary/GaitSchedulerPat.h>
#include <auxillary/LegControllerPat.h>
#include <estimators/StateEstimatorContainer.h>
#include <robots/PatBiped.h>
/**
 *
 */
template <typename T>
struct MoCapData{
   Vec3<T> com_pos;
   Vec3<T> com_vel;
   Vec4<T> marker_orientation; //quaternion
};
template struct MoCapData<float>;
template struct MoCapData<double>;

/**
 *
 */
template <typename T>
struct ControlFSMData {
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int _robot_id;
  PatBiped<T>* _pat = NULL;
  StateEstimatorContainer<T>* _stateEstimator;
  LegControllerPat<T>* _legController;
  GaitSchedulerPat<T>* _gaitScheduler;
  DesiredStateCommand<T>* _desiredStateCommand;
  PatParameters* userParameters;
  VisualizationData* visualizationData;
  MoCapData<T>* mocapData;

};

template struct ControlFSMData<double>;
template struct ControlFSMData<float>;

#endif  // CONTROLFSM_H
