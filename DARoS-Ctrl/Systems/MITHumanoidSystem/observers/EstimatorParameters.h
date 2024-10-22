#ifndef ESTIMATOR_PARAMETERS_H
#define ESTIMATOR_PARAMETERS_H

#include <ControlParameters/ControlParameters.h>

class EstimatorParameters : public ControlParameters {
  public:
  EstimatorParameters()
      : ControlParameters("estimator-parameters"),
        INIT_PARAMETER(estimator_dt),
        INIT_PARAMETER(imu_process_noise_position),
        INIT_PARAMETER(imu_process_noise_velocity),
        INIT_PARAMETER(foot_process_noise_position),
        INIT_PARAMETER(foot_sensor_noise_position),
        INIT_PARAMETER(foot_sensor_noise_velocity),
        INIT_PARAMETER(foot_height_sensor_noise),
        INIT_PARAMETER(mu_p_c),
        INIT_PARAMETER(mu_p_s),
        INIT_PARAMETER(mu_z_g),
        INIT_PARAMETER(mu_f_c),
        INIT_PARAMETER(var_p_c),
        INIT_PARAMETER(var_p_s),
        INIT_PARAMETER(var_z_g),
        INIT_PARAMETER(var_f_c),
        INIT_PARAMETER(var_g_p),
        INIT_PARAMETER(var_p_z),
        INIT_PARAMETER(var_f_z),
        INIT_PARAMETER(transition_threshold),
        INIT_PARAMETER(contact_threshold)
{}

  DECLARE_PARAMETER(double, estimator_dt)
  DECLARE_PARAMETER(double, imu_process_noise_position)
  DECLARE_PARAMETER(double, imu_process_noise_velocity)
  DECLARE_PARAMETER(double, foot_process_noise_position)
  DECLARE_PARAMETER(double, foot_sensor_noise_position)
  DECLARE_PARAMETER(double, foot_sensor_noise_velocity)
  DECLARE_PARAMETER(double, foot_height_sensor_noise)
  DECLARE_PARAMETER(Vec3<double>, mu_p_c)
  DECLARE_PARAMETER(Vec3<double>, mu_p_s)
  DECLARE_PARAMETER(double, mu_z_g)
  DECLARE_PARAMETER(double, mu_f_c)
  DECLARE_PARAMETER(Vec3<double>, var_p_c)
  DECLARE_PARAMETER(Vec3<double>, var_p_s)
  DECLARE_PARAMETER(double, var_z_g)
  DECLARE_PARAMETER(double, var_f_c)
  DECLARE_PARAMETER(double, var_g_p)
  DECLARE_PARAMETER(double, var_p_z)
  DECLARE_PARAMETER(double, var_f_z)
  DECLARE_PARAMETER(double, transition_threshold)
  DECLARE_PARAMETER(double, contact_threshold)
};
#endif
