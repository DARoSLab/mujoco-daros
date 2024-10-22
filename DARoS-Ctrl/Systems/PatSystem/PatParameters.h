/*! @file RobotParameters.cpp
 *  @brief Declaration of various robot parameters
 *
 *  This class contains all the ControlParameters which are shared between all robot controllers
 *  Currently there are some userParameters that are specific to the MIT controllers here,
 *  but these will be moved in the future
 */

#ifndef PROJECT_Pat_PARAMETERS_H
#define PROJECT_Pat_PARAMETERS_H

#include <ControlParameters/ControlParameters.h>

/*!
 * ControlParameters shared between all robot controllers
 */
class PatParameters : public ControlParameters {
 public:

  /*!
   * Construct RobotControlParameters
   */
  PatParameters()
      : ControlParameters("pat-parameters"),
        INIT_PARAMETER(simulation_test),
        INIT_PARAMETER(myValue),
        INIT_PARAMETER(control_mode),
        INIT_PARAMETER(training_control_mode),
        INIT_PARAMETER(testValue),
        INIT_PARAMETER(controller_dt),
        INIT_PARAMETER(stand_kp_cartesian),
        INIT_PARAMETER(stand_kd_cartesian),
        INIT_PARAMETER(kpCOM),
        INIT_PARAMETER(kdCOM),
        INIT_PARAMETER(kpBase),
        INIT_PARAMETER(kdBase),
        INIT_PARAMETER(cheater_mode),
        //INIT_PARAMETER(imu_process_noise_position),
        //INIT_PARAMETER(imu_process_noise_velocity),
        //INIT_PARAMETER(foot_process_noise_position),
        //INIT_PARAMETER(foot_sensor_noise_position),
        //INIT_PARAMETER(foot_sensor_noise_velocity),
        //INIT_PARAMETER(foot_height_sensor_noise),
        INIT_PARAMETER(use_rc),
        INIT_PARAMETER(PATH_move_waypoint),
        INIT_PARAMETER(cmpc_gait),
        INIT_PARAMETER(cmpc_x_drag),
        INIT_PARAMETER(cmpc_use_sparse),
        INIT_PARAMETER(use_wbc),
        INIT_PARAMETER(cmpc_bonus_swing),
        INIT_PARAMETER(Kp_body),
        INIT_PARAMETER(Kd_body),
        INIT_PARAMETER(Kp_ori),
        INIT_PARAMETER(Kd_ori),
        INIT_PARAMETER(Kp_foot),
        INIT_PARAMETER(Kd_foot),
        INIT_PARAMETER(Kp_joint),
        INIT_PARAMETER(Kd_joint),
        INIT_PARAMETER(Kp_cam),
        INIT_PARAMETER(Kd_cam),
        INIT_PARAMETER(Kp_clm),
        INIT_PARAMETER(Kd_clm),
        INIT_PARAMETER(Q_pos),
        INIT_PARAMETER(Q_vel),
        INIT_PARAMETER(Q_ori),
        INIT_PARAMETER(Q_ang),
        INIT_PARAMETER(Ig_mc),
        INIT_PARAMETER(R_control),
        INIT_PARAMETER(R_prev),
        INIT_PARAMETER(N_height),
        INIT_PARAMETER(yaw_bias),
        INIT_PARAMETER(friction_coef),
        INIT_PARAMETER(max_ori_error),
        INIT_PARAMETER(Q_pos_loc),
        INIT_PARAMETER(Q_vel_loc),
        INIT_PARAMETER(Q_ori_loc),
        INIT_PARAMETER(Q_ang_loc),
        INIT_PARAMETER(Ig_mc_loc),
        INIT_PARAMETER(R_control_loc),
        INIT_PARAMETER(R_prev_loc),
        INIT_PARAMETER(N_height_loc),
        INIT_PARAMETER(yaw_bias_loc),
        INIT_PARAMETER(friction_coef_loc),
        INIT_PARAMETER(max_ori_error_loc),
        INIT_PARAMETER(stance_legs),
        INIT_PARAMETER(acro_task),
        INIT_PARAMETER(use_jcqp),
        INIT_PARAMETER(jcqp_max_iter),
        INIT_PARAMETER(jcqp_rho),
        INIT_PARAMETER(jcqp_sigma),
        INIT_PARAMETER(jcqp_alpha),
        INIT_PARAMETER(jcqp_terminate),
        INIT_PARAMETER(Swing_Kp_cartesian),
        INIT_PARAMETER(Swing_Kd_cartesian),
        INIT_PARAMETER(Swing_Kp_joint),
        INIT_PARAMETER(Swing_Kd_joint),
        INIT_PARAMETER(Swing_step_offset),
        INIT_PARAMETER(Swing_traj_height),
        INIT_PARAMETER(Swing_use_tau_ff),
        INIT_PARAMETER(RPC_Q_p),
        INIT_PARAMETER(RPC_Q_theta),
        INIT_PARAMETER(RPC_Q_dp),
        INIT_PARAMETER(RPC_Q_dtheta),
        INIT_PARAMETER(RPC_R_r),
        INIT_PARAMETER(RPC_R_f),
        INIT_PARAMETER(RPC_H_r_trans),
        INIT_PARAMETER(RPC_H_r_rot),
        INIT_PARAMETER(RPC_H_theta0),
        INIT_PARAMETER(RPC_H_phi0),
        INIT_PARAMETER(RPC_mass),
        INIT_PARAMETER(RPC_inertia),
        INIT_PARAMETER(RPC_gravity),
        INIT_PARAMETER(RPC_mu),
        INIT_PARAMETER(RPC_filter),
        INIT_PARAMETER(RPC_use_pred_comp),
        INIT_PARAMETER(RPC_use_async_filt),
        INIT_PARAMETER(RPC_visualize_pred),
        INIT_PARAMETER(RPC_interface_type),
        INIT_PARAMETER(des_p),
        INIT_PARAMETER(des_theta),
        INIT_PARAMETER(des_dp),
        INIT_PARAMETER(des_dtheta),
        INIT_PARAMETER(des_theta_max),
        INIT_PARAMETER(des_dp_max),
        INIT_PARAMETER(des_dtheta_max),
        INIT_PARAMETER(gait_type),
        INIT_PARAMETER(gait_period_time),
        INIT_PARAMETER(gait_switching_phase),
        INIT_PARAMETER(gait_override),
        INIT_PARAMETER(gait_max_leg_angle),
        INIT_PARAMETER(gait_max_stance_time),
        INIT_PARAMETER(gait_min_stance_time),
        INIT_PARAMETER(gait_disturbance),
        INIT_PARAMETER(gait_recovery),
        INIT_PARAMETER(kpx)
        // INIT_PARAMETER(kpy),
        // INIT_PARAMETER(kdx),
        // INIT_PARAMETER(kdy)

  {}

  DECLARE_PARAMETER(s64, simulation_test)
  DECLARE_PARAMETER(double, myValue)
  DECLARE_PARAMETER(double, control_mode)
  DECLARE_PARAMETER(double, training_control_mode)
  DECLARE_PARAMETER(double, testValue)
  DECLARE_PARAMETER(double, controller_dt)
  DECLARE_PARAMETER(Vec3<double>, stand_kp_cartesian)
  DECLARE_PARAMETER(Vec3<double>, stand_kd_cartesian)
  DECLARE_PARAMETER(Vec3<double>, kpCOM)
  DECLARE_PARAMETER(Vec3<double>, kdCOM)
  DECLARE_PARAMETER(Vec3<double>, kpBase)
  DECLARE_PARAMETER(Vec3<double>, kdBase)

  // state estimator
  DECLARE_PARAMETER(s64, cheater_mode)
  //DECLARE_PARAMETER(double, imu_process_noise_position)
  //DECLARE_PARAMETER(double, imu_process_noise_velocity)
  //DECLARE_PARAMETER(double, foot_process_noise_position)
  //DECLARE_PARAMETER(double, foot_sensor_noise_position)
  //DECLARE_PARAMETER(double, foot_sensor_noise_velocity)
  //DECLARE_PARAMETER(double, foot_height_sensor_noise)

  DECLARE_PARAMETER(double, use_rc)

  DECLARE_PARAMETER(double, PATH_move_waypoint)

  DECLARE_PARAMETER(double, cmpc_gait);
  DECLARE_PARAMETER(double, cmpc_x_drag);
  DECLARE_PARAMETER(double, cmpc_use_sparse);
  DECLARE_PARAMETER(double, use_wbc);
  DECLARE_PARAMETER(double, cmpc_bonus_swing);

  DECLARE_PARAMETER(Vec3<double>, Kp_body);
  DECLARE_PARAMETER(Vec3<double>, Kd_body);

  DECLARE_PARAMETER(Vec3<double>, Kp_ori);
  DECLARE_PARAMETER(Vec3<double>, Kd_ori);

  DECLARE_PARAMETER(Vec3<double>, Kp_foot);
  DECLARE_PARAMETER(Vec3<double>, Kd_foot);

  DECLARE_PARAMETER(Vec3<double>, Kp_joint);
  DECLARE_PARAMETER(Vec3<double>, Kd_joint);

  DECLARE_PARAMETER(Vec3<double>, Kp_cam);
  DECLARE_PARAMETER(Vec3<double>, Kd_cam);

  DECLARE_PARAMETER(Vec3<double>, Kp_clm);
  DECLARE_PARAMETER(Vec3<double>, Kd_clm);

  DECLARE_PARAMETER(Vec3<double>, Q_pos);
  DECLARE_PARAMETER(Vec3<double>, Q_vel);
  DECLARE_PARAMETER(Vec3<double>, Q_ori);
  DECLARE_PARAMETER(Vec3<double>, Q_ang);
  DECLARE_PARAMETER(Vec3<double>, Ig_mc);
  DECLARE_PARAMETER(double, R_control);
  DECLARE_PARAMETER(double, R_prev);
  DECLARE_PARAMETER(double, N_height);
  DECLARE_PARAMETER(double, yaw_bias);
  DECLARE_PARAMETER(double, friction_coef);
  DECLARE_PARAMETER(double, max_ori_error);
  DECLARE_PARAMETER(Vec3<double>, Q_pos_loc);
  DECLARE_PARAMETER(Vec3<double>, Q_vel_loc);
  DECLARE_PARAMETER(Vec3<double>, Q_ori_loc);
  DECLARE_PARAMETER(Vec3<double>, Q_ang_loc);
  DECLARE_PARAMETER(Vec3<double>, Ig_mc_loc);
  DECLARE_PARAMETER(double, R_control_loc);
  DECLARE_PARAMETER(double, R_prev_loc);
  DECLARE_PARAMETER(double, N_height_loc);
  DECLARE_PARAMETER(double, yaw_bias_loc);
  DECLARE_PARAMETER(double, friction_coef_loc);
  DECLARE_PARAMETER(double, max_ori_error_loc);
  DECLARE_PARAMETER(double, stance_legs);

  DECLARE_PARAMETER(double, acro_task);

  DECLARE_PARAMETER(double, use_jcqp);
  DECLARE_PARAMETER(double, jcqp_max_iter);
  DECLARE_PARAMETER(double, jcqp_rho);
  DECLARE_PARAMETER(double, jcqp_sigma);
  DECLARE_PARAMETER(double, jcqp_alpha);
  DECLARE_PARAMETER(double, jcqp_terminate);

  // Swing leg parameters
  DECLARE_PARAMETER(Vec3<double>, Swing_Kp_cartesian);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kd_cartesian);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kp_joint);
  DECLARE_PARAMETER(Vec3<double>, Swing_Kd_joint);
  DECLARE_PARAMETER(Vec3<double>, Swing_step_offset);
  DECLARE_PARAMETER(double, Swing_traj_height);
  DECLARE_PARAMETER(double, Swing_use_tau_ff);


  // Parameters used for RPC
  DECLARE_PARAMETER(Vec3<double>, RPC_Q_p);
  DECLARE_PARAMETER(Vec3<double>, RPC_Q_theta);
  DECLARE_PARAMETER(Vec3<double>, RPC_Q_dp);
  DECLARE_PARAMETER(Vec3<double>, RPC_Q_dtheta);
  DECLARE_PARAMETER(Vec3<double>, RPC_R_r);
  DECLARE_PARAMETER(Vec3<double>, RPC_R_f);
  DECLARE_PARAMETER(Vec3<double>, RPC_H_r_trans);
  DECLARE_PARAMETER(Vec3<double>, RPC_H_r_rot);
  DECLARE_PARAMETER(Vec3<double>, RPC_H_theta0);
  DECLARE_PARAMETER(Vec3<double>, RPC_H_phi0);
  DECLARE_PARAMETER(double, RPC_mass);
  DECLARE_PARAMETER(Vec3<double>, RPC_inertia);
  DECLARE_PARAMETER(Vec3<double>, RPC_gravity);
  DECLARE_PARAMETER(double, RPC_mu);
  DECLARE_PARAMETER(Vec3<double>, RPC_filter);
  DECLARE_PARAMETER(double, RPC_use_pred_comp);
  DECLARE_PARAMETER(double, RPC_use_async_filt);
  DECLARE_PARAMETER(double, RPC_visualize_pred);
  DECLARE_PARAMETER(double, RPC_interface_type);

  // Desired states
  DECLARE_PARAMETER(Vec3<double>, des_p);
  DECLARE_PARAMETER(Vec3<double>, des_theta);
  DECLARE_PARAMETER(Vec3<double>, des_dp);
  DECLARE_PARAMETER(Vec3<double>, des_dtheta);
  DECLARE_PARAMETER(Vec3<double>, des_theta_max);
  DECLARE_PARAMETER(Vec3<double>, des_dp_max);
  DECLARE_PARAMETER(Vec3<double>, des_dtheta_max);

  // Gait Scheduler
  DECLARE_PARAMETER(double, gait_type);
  DECLARE_PARAMETER(double, gait_period_time);
  DECLARE_PARAMETER(double, gait_switching_phase);
  DECLARE_PARAMETER(double, gait_override);
  DECLARE_PARAMETER(double, gait_max_leg_angle);
  DECLARE_PARAMETER(double, gait_max_stance_time);
  DECLARE_PARAMETER(double, gait_min_stance_time);
  DECLARE_PARAMETER(Vec3<double>, gait_disturbance);
  DECLARE_PARAMETER(Vec3<double>, gait_recovery);
  DECLARE_PARAMETER(double, kpx);
  // DECLARE_PARAMETER(double, kpy);
  // DECLARE_PARAMETER(double, kdx);
  // DECLARE_PARAMETER(double, kdy);
};

#endif  // PROJECT_Pat_PARAMETERS_H
