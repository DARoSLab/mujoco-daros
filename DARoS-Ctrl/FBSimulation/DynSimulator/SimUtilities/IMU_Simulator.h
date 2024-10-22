/*! @file IMU_Simulator.h
 *  @brief Simulated IMU with noise
 */

#ifndef PROJECT_IMUSIMULATOR_H
#define PROJECT_IMUSIMULATOR_H

#include <random>

#include "ControlParameters/SimulatorParameters.h"
#include <FloatingBaseModel.h>
#include "cppTypes.h"

/*!
 * Simulation of IMU
 */
template <typename T>
class IMU_Simulator {
 public:
  explicit IMU_Simulator(SimulatorParameters& simSettings, u64 seed = 0)
      : _simSettings(simSettings),
        _mt(seed),
        _IMU_GyroDistribution(-simSettings.imu_gyro_noise,
                                   simSettings.imu_gyro_noise),
        _IMU_AccelerometerDistribution(
            -simSettings.imu_accelerometer_noise,
            simSettings.imu_accelerometer_noise),
        _IMU_QuatDistribution(-simSettings.imu_quat_noise,
                                   simSettings.imu_quat_noise) {
    if (simSettings.imu_quat_noise != 0) {
      _IMU_OrientationNoise = true;
    }
  }

  void updateIMU(const FBModelState<T>& robotState,
      const FBModelStateDerivative<T>& robotStateD,
      IMU_Data* data);

  void updateCheaterState(const FBModelState<T>& robotState,
      const FBModelStateDerivative<T>& robotStateD,
      CheaterState& state);

 private:
  void _computeAcceleration(const FBModelState<T>& robotState,
                           const FBModelStateDerivative<T>& robotStateD,
                           Vec3<float>& acc,
                           std::uniform_real_distribution<float>& dist,
                           const RotMat<float>& R_body);

  SimulatorParameters& _simSettings;
  std::mt19937 _mt;
  std::uniform_real_distribution<float> _IMU_GyroDistribution;
  std::uniform_real_distribution<float> _IMU_AccelerometerDistribution;
  std::uniform_real_distribution<float> _IMU_QuatDistribution;
  bool _IMU_OrientationNoise = false;
};
#endif  // PROJECT_IMUSIMULATOR_H
