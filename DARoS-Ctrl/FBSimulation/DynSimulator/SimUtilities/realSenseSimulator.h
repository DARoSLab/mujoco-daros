/*! @file realSenseSimulator.h
 *  @brief Simulated RealSense with noise
 */

#ifndef PROJECT_REALSENSESIMULATOR_H
#define PROJECT_REALSENSESIMULATOR_H

#include <random>
#include "ControlParameters/SimulatorParameters.h"
#include <FloatingBaseModel.h>
#include "cppTypes.h"

#define NUM_TRACKING_SENSOR 2

struct LocalizationData {
  // position of the localization sensor represented in the real sense inertial (fixed) frame
  Vec3<float> position[NUM_TRACKING_SENSOR]; 
  // position of the robot represented in the standard world frame  
  Vec3<float> positionRobot[NUM_TRACKING_SENSOR]; 
  Vec3<float> vBody[NUM_TRACKING_SENSOR];
  Vec3<float> acceleration[NUM_TRACKING_SENSOR];
  Vec3<float> omegaBody[NUM_TRACKING_SENSOR];
  Mat3<float> R_rsInertial_to_rsRelative[NUM_TRACKING_SENSOR]; // describes rotation from real sense inertial (fixed) frame to the real sense (relative) frame
  Vec3<float> rpy_rsInertial_to_rsRelative[NUM_TRACKING_SENSOR];
  Vec3<float> rpy_world_to_body[NUM_TRACKING_SENSOR];
  Quat<float> orientation[NUM_TRACKING_SENSOR]; 
  float tracker_conf[NUM_TRACKING_SENSOR];
  float mapper_conf[NUM_TRACKING_SENSOR];
};


/*!
 * Simulation of Real Sense
 */
template <typename T>
class realSenseSimulator {
 public:
  explicit realSenseSimulator(SimulatorParameters& simSettings, u64 seed = 0)
      : _simSettings(simSettings),
        _mt(seed),
        _localizationPositionDistribution(-simSettings.rs_localization_position_noise,
                                   simSettings.rs_localization_position_noise),
        _localizationOrientationDistribution(-simSettings.rs_localization_position_noise,
                                   simSettings.rs_localization_position_noise)
  {
    if (simSettings.rs_localization_position_noise + simSettings.rs_localization_orientation_noise != 0) {
      _localizationNoise = true;
    }
  }

  void updateLocalization(const FBModelState<T>& robotState,
                       const FBModelStateDerivative<T>& robotStateD,
                       LocalizationData* data);

 private:
  SimulatorParameters& _simSettings;
  std::mt19937 _mt;
  std::uniform_real_distribution<float> _localizationPositionDistribution;
  std::uniform_real_distribution<float> _localizationOrientationDistribution;
  bool _localizationNoise = false;

};
#endif  // PROJECT_REALSENSESIMULATOR_H
