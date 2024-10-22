/*! @file JointControl.h
 *  @brief Common Joint Control Interface and Joint Control Algorithms
 *
 *  Implements low-level limb control for te MIT Humanoid Robots
 *  All quantities are in the "limb frame" which has the same orientation as the
 *  body frame, but is shifted so that 0,0,0 is at the first pivot of the limb
 *  (the "hip z frame" for legs and "shoulder x frame" for arms).
 */

#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include "cppTypes.h"
#include <Utilities/pretty_print.h>
/*!
 * Data sent from the control algorithm to the robot
 */
template <typename T>
struct JointControlCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  JointControlCommand(int num_joints){
    _num_joints = num_joints;
    tauFeedForward = DVec<T>::Zero(_num_joints);
    qDes = DVec<T>::Zero(_num_joints);
    qdDes = DVec<T>::Zero(_num_joints);
    kpJoint = DMat<T>::Zero(_num_joints, _num_joints);
    kdJoint = DMat<T>::Zero(_num_joints, _num_joints);
  }
  ~JointControlCommand(){}

  void zero(){
    tauFeedForward.setZero();
    qDes.setZero();
    qdDes.setZero();
    kpJoint.setZero();
    kdJoint.setZero();
  }

  int _num_joints;
  DVec<T> tauFeedForward,  qDes, qdDes;
  DMat<T> kpJoint, kdJoint;
};

/*!
 * Data returned from the limbs to the control code.
 */
template <typename T>
struct JointControlData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  JointControlData(int num_joints){
    _num_joints = num_joints;
    q = DVec<T>::Zero(_num_joints);
    qd = DVec<T>::Zero(_num_joints);
    tauEstimate = DVec<T>::Zero(_num_joints);
    tauAct = DVec<T>::Zero(_num_joints);
  }
  ~JointControlData(){}
  void zero(){
    q.setZero();
    qd.setZero();
    tauEstimate.setZero();
    tauAct.setZero();
  }

  DVec<T> tauEstimate, tauAct;
  DVec<T> q, qd;
  Vec3<T> p, v; // Limb End point position/ velocity
  DMat<T> J; // Jacobian
  int _num_joints;
};

#endif  // JOINT_CONTROL_H
