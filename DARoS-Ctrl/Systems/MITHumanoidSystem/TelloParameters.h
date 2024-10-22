/*! @file RobotParameters.cpp
 *  @brief Declaration of various robot parameters
 *
 *  This class contains all the ControlParameters which are shared between all robot controllers
 *  Currently there are some userParameters that are specific to the MIT controllers here,
 *  but these will be moved in the future
 */

#ifndef TELLO_PARAMETERS_H
#define TELLO_PARAMETERS_H

#include <ControlParameters/ControlParameters.h>

/*!
 * ControlParameters shared between all robot controllers
 */
class TelloParameters : public ControlParameters {
 public:

  /*!
   * Construct RobotControlParameters
   */
  TelloParameters()
      : ControlParameters("tello-parameters"),
        INIT_PARAMETER(myValue),
        INIT_PARAMETER(control_mode),
        INIT_PARAMETER(controller_dt),
        INIT_PARAMETER(use_rc),
        INIT_PARAMETER(Kp_ori),
        INIT_PARAMETER(Kd_ori),
        INIT_PARAMETER(Kp_body),
        INIT_PARAMETER(Kd_body),
        INIT_PARAMETER(Kp_foot),
        INIT_PARAMETER(Kd_foot),
        INIT_PARAMETER(Kp_joint),
        INIT_PARAMETER(Kd_joint),
        INIT_PARAMETER(gravity),
        INIT_PARAMETER(mu),
        INIT_PARAMETER(gait_num_segments),
        INIT_PARAMETER(gait_right_offset),
        INIT_PARAMETER(gait_left_offset),
        INIT_PARAMETER(gait_duration),
        INIT_PARAMETER(gait_swing_height),
        INIT_PARAMETER(stairstep_height),
        INIT_PARAMETER(gait_toe_offset),
        INIT_PARAMETER(gait_heel_offset),
        INIT_PARAMETER(gait_foot_offset),
        INIT_PARAMETER(gait_vel_gain),
        INIT_PARAMETER(CAS_dt),
        INIT_PARAMETER(CAS_pred_hor),
        INIT_PARAMETER(CAS_x_des),
        INIT_PARAMETER(CAS_v_des),
        INIT_PARAMETER(CAS_w_des),
        INIT_PARAMETER(CAS_RPY_des),
        INIT_PARAMETER(CAS_Q_X),
        INIT_PARAMETER(CAS_Q_Xd),
        INIT_PARAMETER(CAS_Q_R),
        INIT_PARAMETER(CAS_Q_W),
        INIT_PARAMETER(CAS_Q_U),
        INIT_PARAMETER(CAS_opt_details),
        INIT_PARAMETER(MAX_REACTION_FORCE),
        INIT_PARAMETER(FloatingBaseWeight),
        INIT_PARAMETER(REACTION_FORCE_Z_WEIGHT),
        INIT_PARAMETER(REACTION_FORCE_X_WEIGHT)
  {}

  DECLARE_PARAMETER(double, myValue)
  DECLARE_PARAMETER(double, control_mode)
  DECLARE_PARAMETER(double, controller_dt)
  // state estimator
  DECLARE_PARAMETER(double, use_rc)

  DECLARE_PARAMETER(Vec3<double>, Kp_ori);
  DECLARE_PARAMETER(Vec3<double>, Kd_ori);
  DECLARE_PARAMETER(Vec3<double>, Kp_body);
  DECLARE_PARAMETER(Vec3<double>, Kd_body);
  DECLARE_PARAMETER(Vec3<double>, Kp_foot);
  DECLARE_PARAMETER(Vec3<double>, Kd_foot);
  DECLARE_PARAMETER(double, Kp_joint);
  DECLARE_PARAMETER(double, Kd_joint);

  DECLARE_PARAMETER(Vec3<double>, gravity);
  DECLARE_PARAMETER(double, mu);


  // Gait Scheduler
  DECLARE_PARAMETER(s64, gait_num_segments);
  DECLARE_PARAMETER(s64, gait_right_offset);
  DECLARE_PARAMETER(s64, gait_left_offset);
  DECLARE_PARAMETER(s64, gait_duration);
  DECLARE_PARAMETER(double, gait_swing_height);
  DECLARE_PARAMETER(double, stairstep_height);
  DECLARE_PARAMETER(double, gait_toe_offset);
  DECLARE_PARAMETER(double, gait_heel_offset);
  DECLARE_PARAMETER(double, gait_foot_offset);
  DECLARE_PARAMETER(float, gait_vel_gain);

  //Casadi parameters
  DECLARE_PARAMETER(double, CAS_dt);
  DECLARE_PARAMETER(s64, CAS_pred_hor);
  DECLARE_PARAMETER(Vec3<float>, CAS_x_des);
  DECLARE_PARAMETER(Vec3<float>, CAS_v_des);
  DECLARE_PARAMETER(Vec3<float>, CAS_w_des);
  DECLARE_PARAMETER(Vec3<float>, CAS_RPY_des);
  DECLARE_PARAMETER(Vec3<double>, CAS_Q_X);
  DECLARE_PARAMETER(Vec3<double>, CAS_Q_Xd);
  DECLARE_PARAMETER(Vec3<double>, CAS_Q_R);
  DECLARE_PARAMETER(Vec3<double>, CAS_Q_W);
  DECLARE_PARAMETER(double, CAS_Q_U);
  DECLARE_PARAMETER(double, CAS_opt_details);
  DECLARE_PARAMETER(double, MAX_REACTION_FORCE);  

  // wbc parameters
  DECLARE_PARAMETER(double, FloatingBaseWeight);  
  DECLARE_PARAMETER(double, REACTION_FORCE_Z_WEIGHT);  
  DECLARE_PARAMETER(double, REACTION_FORCE_X_WEIGHT); 


};

#endif  // TELLO_PARAMETERS_H
