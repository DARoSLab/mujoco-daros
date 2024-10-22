/*! @file PatBiped.h
 *  @brief Data structure containing parameters for pat robot
 *
 *  This file contains the Pat class.  This stores all the parameters for
 * a pat robot.  There are utility functions to generate Pat objects
 * for Cheetah 3 (and eventually mini-pat). There is a buildModel() method
 * which can be used to create a floating-base dynamics model of the pat.
 */

#ifndef POINT_FOOT_BIP_H
#define POINT_FOOT_BIP_H

#include <vector>
#include <Robot.h>
#include <ActuatorModel.h>
#include <FloatingBaseModel.h>
#include <utils/Utilities/SpatialInertia.h>
#include "PatBipedActuatorModel.h"
#include <eigen3/Eigen/StdVector>

/*!
 * Basic parameters for a pat-shaped robot
 */
namespace pat_biped
{
  constexpr size_t num_act_joints = 6;
  constexpr size_t num_q = 13;
  constexpr size_t dim_config = 12;
  constexpr size_t num_legs = 2;
  constexpr size_t num_limbs = 2;
  constexpr size_t num_legs_joint = 3;
} // namespace pat

/*!
 * Link indices for pat-shaped robots
 */
namespace pat_biped_linkID
{
  constexpr size_t RF = 10; // Right Foot
  constexpr size_t LF = 13; // Left Foot
  constexpr size_t RS = 8; // Right Shank
  constexpr size_t LS = 11; // Left Shank


  constexpr size_t R_abd = 2; // Right Abduction
  constexpr size_t L_abd = 0; // Left Abduction
}
// namespace pat_linkID

/*!
 * Representation of a pat robot's physical properties.
 *
 * When viewed from the top, the pat's legs are:
 *
 * FRONT
 * 2 1   RIGHT
 * BACK
 *
 */
template <typename T>
class PatBiped : public Robot<T>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PatBiped() : Robot<T>()
  {
    this->NUM_FEET = pat_biped::num_legs;
    this->NUM_CONTROLLED_LIMBS = pat_biped::num_limbs;
    this->limb_contact_points = DiVec::Zero(this->NUM_CONTROLLED_LIMBS);
    this->limb_contact_points << 1, 1;
    this->NUM_CONTACT_POINTS = this->limb_contact_points.sum();
  }
  virtual ~PatBiped() {}

  T _bodyLength, _bodyWidth, _bodyHeight, _bodyMass;
  T _abadGearRatio, _hipGearRatio, _kneeGearRatio;
  T _abadLinkLength, _abadLinkZ_offset, _abadLinkX_offset, _hipLinkY_offset, _hipLinkLength, _kneeLinkLength, _kneeLinkY_offset, _kneeLinkX_offset, _maxLegLength;
  T _motorKT, _motorR, _batteryV;
  T _motorTauMax;
  T _jointDamping, _jointDryFriction;
  T _abdMotorKT, _abdMotorR;
  T _abdMotorTauMax;
  T _abdJointDamping, _abdJointDryFriction;

  T _x_offset;
  SpatialInertia<T> _abadInertia, _hipInertia, _kneeInertia, _abadRotorInertia,
      _hipRotorInertia, _kneeRotorInertia, _bodyInertia;
  Vec3<T> _abadLocation, _abadRotorLocation, _hipLocation, _hipRotorLocation,
      _kneeLocation, _kneeRotorLocation, _footLocation;

  CoordinateAxis _abad_axis, _hip_axis, _knee_axis;

  JointType _abad_joint_type, _hip_joint_type, _knee_joint_type;

  // Must be set in the individual robot
  FBModelState<T> _x_initial;
  FBModelState<T> _x_home;

  virtual FloatingBaseModel<T> buildModel(T gravity = -9.81) const;
  virtual std::vector<ActuatorModel<T> *> buildActuatorModels() const;
  virtual void getInitialState(FBModelState<T> &x0) const { x0 = _x_initial; }
  virtual void getHomeState(FBModelState<T> &xHome) const { xHome = _x_home; }
  virtual int numActJoint() const { return pat_biped::num_act_joints; }

  /*!
   * Get if the i-th leg is on the left (+) or right (-) of the robot.
   * @param leg : the leg index
   * @return The side sign (-1 for right legs, +1 for left legs)
   */
  static T getSideSign(int leg)
  {
    const T sideSigns[2] = {-1, 1};
    assert(leg >= 0 && leg < 2);
    return sideSigns[leg];
  }

  /*!
   * Get location of the hip for the given leg in robot frame
   * @param leg : the leg index
   */
  virtual Vec3<T> getHipLocation(int leg) const
  {
    assert(leg >= 0 && leg < 2);
    Vec3<T> pHip(
        _abadLocation(0),
        (leg == 1) ? _abadLocation(1) : -_abadLocation(1),
        _abadLocation(2));
    return pHip;
  }
};

template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2> &v, int legID);

#endif // LIBBIOMIMETICS_POINT_FOOT_BIP_H
