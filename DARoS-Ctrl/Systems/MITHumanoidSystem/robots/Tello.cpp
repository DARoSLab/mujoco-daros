/*! @file Tello.cpp
 *  @brief Data structure containing parameters for tello robot
 *
 *  This file contains the Tello class.  This stores all the parameters for
 *  a tello robot.  There are utility functions to generate  objects.
 *  There is a buildModel() method which can be used to create a floating-base
 *  dynamics model of the tello.
 */

#include "Tello.h"
#include <ParamHandler/ParamHandler.hpp>

#include <Utilities/spatial.h>
#include <Utilities/orientation_tools.h>
#include <Utilities/pretty_print.h>
#include <Utilities/SpatialInertia.h>
#include <Configuration.h>

#include <urdf/model.h>
#include <urdf/urdf_parser.h>

#include <Dynamics/parsingURDF.h>



/*!
 * Build a FloatingBaseModel of the tello
 */
template <typename T>
FloatingBaseModel<T> Tello<T>::buildModel(T gravity) const {
  FloatingBaseModel<T> model;

  buildFloatingBaseModelFromURDF(model, THIS_COM"Systems/MIT_Humanoid/tello_config/humanoid_v4.urdf", true);

  // Contact setup

  Vec3<T> offset; offset.setZero();
  offset[2] = _waistDims[2]/2.;
  model.addGroundContactBoxPointsOffset(5, _waistDims, offset);

  // offset[2] = 0.3 - _torsoDims[2]/2.;
  // model.addGroundContactBoxPointsOffset(tello_link::torso, _torsoDims, offset);
  model.addGroundContactPoint(tello_link::R_foot, Vec3<T>(-0.05, 0., -0.041));
  model.addGroundContactPoint(tello_link::R_foot, Vec3<T>(0.1, 0.0, -0.041));

  model.addGroundContactPoint(tello_link::L_foot, Vec3<T>(-0.05, 0.0, -0.041));
  model.addGroundContactPoint(tello_link::L_foot, Vec3<T>(0.1, 0.0, -0.041));


  Vec3<T> g(0, 0, gravity);
  model.setGravity(g);
  return model;
}


/*!
 * Build actuator models for legs and arms
 */
template <typename T>
std::vector<ActuatorModel<T>*> Tello<T>::buildActuatorModels() const {
  std::vector<ActuatorModel<T>*> models;

  // Right & Left legs
  for(size_t i(0); i<tello::num_act_joint; ++i){
    TelloActuatorModel<T>* actuator = new TelloActuatorModel<T>(_GearRatio,
      _smallMotorKT, _smallMotorR, _batteryV, _jointDamping, _jointDryFriction, 
      _smallMotorTauMax, _smallMotorFluxLinkage, _smallMotorInductance, _smallMotorPoles);
    models.push_back(actuator);
  }
  return models;
}


template <typename T>
void Tello<T>::getInitialState(FBModelState<T> & x0) const {
  x0.bodyPosition.setZero();
  x0.bodyVelocity.setZero();
  x0.bodyOrientation.setZero();
  x0.bodyOrientation[0] = 1.;
  x0.q = DVec<T>::Zero(tello::num_act_joint);
  x0.qd = DVec<T>::Zero(tello::num_act_joint);

  std::vector<T> vec;
  ParamHandler param(THIS_COM"Systems/MIT_Humanoid/tello_config/tello-configuration.yaml");
  param.getVector("initial_rpy", vec);
  Vec3<T> ini_rpy(vec.data());

  param.getVector("initial_body_vel", vec);
  SVec<T> ini_vel(vec.data());

  param.getVector("initial_body_pos", vec);
  Vec3<T> ini_pos(vec.data());

  param.getVector("initial_jpos", vec);
  DVec<T> ini_jpos(tello::num_act_joint);

  for(size_t i(0); i<tello::num_act_joint; ++i){
    ini_jpos[i] = vec[i];
  }

  x0.bodyOrientation = rotationMatrixToQuaternion(ori::rpyToRotMat(ini_rpy));
  x0.bodyPosition = ini_pos;
  x0.bodyVelocity = ini_vel;

  for(size_t i(0); i<tello::num_act_joint; ++i){
    x0.q[i] = ini_jpos[i];
  }

  //pretty_print(ini_rpy, std::cout, "ini rpy");
  //pretty_print(ini_vel, std::cout, "ini body vel");
  //pretty_print(ini_pos, std::cout, "ini pos");
  //pretty_print(ini_jpos, std::cout, "ini jpos");
}

template class Tello<double>;
template class Tello<float>;
