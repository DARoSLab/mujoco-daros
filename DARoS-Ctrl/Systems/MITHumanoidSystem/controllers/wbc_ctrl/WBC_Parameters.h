/*! @file RobotParameters.cpp
 *  @
 *
 *  This class contains all the ControlParameters for a wbic controller setup file
 */

#ifndef PROJECT_WBC_PARAMETERS_H
#define PROJECT_WBC_PARAMETERS_H

#include <ControlParameters/ControlParameters.h>

/*!
 * ControlParameters shared between all robot controllers
 */
class WBCParameters : public ControlParameters {
 public:

  /*!
   * Construct RobotControlParameters
   */
  WBCParameters()
      : ControlParameters("wbc-parameters"),
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
        INIT_PARAMETER(priority)

  {}

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
  DECLARE_PARAMETER(double, priority); // priority flag

};

#endif  // PROJECT_WBC_PARAMETERS_H
