/*! @file PatBiped.cpp
 *  @brief Data structure containing parameters for guide dog robot
 *
 *  This file contains the PatBiped class.  This stores all the parameters for
 * a guide dog robot.  There are utility functions to generate PatBiped objects
 * for Cheetah 3 (and eventually mini-cheetah). There is a buildModel() method
 * which can be used to create a floating-base dynamics model of the guide dog.
 */

#include "PatBiped.h"
#include <Utilities/spatial.h>
#include <Utilities/orientation_tools.h>

using namespace ori;
using namespace spatial;

/*!
 * Build a FloatingBaseModel of the guide dog
 */
template <typename T>
FloatingBaseModel<T> PatBiped<T>::buildModel(T gravity) const
{
  FloatingBaseModel<T> model;
  // we assume the cheetah's body (not including rotors) can be modeled as a
  // uniformly distributed box.
  Vec3<T> bodyDims(_bodyLength, _bodyWidth, _bodyHeight);
  // model.addBase(_bodyMass, Vec3<T>(0,0,0), rotInertiaOfBox(_bodyMass,
  // bodyDims));
  model.addBase(_bodyInertia);
  // add contact for the cheetah's body
  // model.addGroundContactBoxPoints(5, bodyDims);

  Vec3<T> offset;
  offset.setZero();
  offset[0] = 0; //_x_offset;
  offset[2] = 0.15;
  model.addGroundContactBoxPointsOffset(5, bodyDims, offset);

  const int baseID = 5;
  int bodyID = baseID;
  T sideSign = -1;

  Mat3<T> I3 = Mat3<T>::Identity();

  // loop over 2 legs
  for (int legID = 0; legID < this->NUM_FEET; legID++)
  {
    // Ab/Ad joint
    //  int addBody(const SpatialInertia<T>& inertia, const SpatialInertia<T>&
    //  rotorInertia, T gearRatio,
    //              int parent, JointType jointType, CoordinateAxis jointAxis,
    //              const Mat6<T>& Xtree, const Mat6<T>& Xrot);
    bodyID++;
    Mat6<T> xtreeAbad = createSXform(I3, withLegSigns<T>(_abadLocation, legID));
    Mat6<T> xtreeAbadRotor = createSXform(I3, withLegSigns<T>(_abadRotorLocation, legID));

    model.addBody(orient_inertia(_abadInertia, sideSign), orient_inertia(_abadRotorInertia, sideSign),
                  _abadGearRatio, baseID, _abad_joint_type,
                  _abad_axis, xtreeAbad, xtreeAbadRotor);

    // Hip Joint
    bodyID++;
    Mat6<T> xtreeHip = createSXform(I3, withLegSigns<T>(_hipLocation, legID));
    Mat6<T> xtreeHipRotor = createSXform(I3, withLegSigns<T>(_hipRotorLocation, legID));

    model.addBody(orient_inertia(_hipInertia, sideSign), orient_inertia(_hipRotorInertia, sideSign),
                  _hipGearRatio, bodyID - 1, _hip_joint_type,
                  _hip_axis, xtreeHip, xtreeHipRotor);

    model.addGroundContactPoint(bodyID, Vec3<T>(0, sideSign * 0.0, 0));

    // add knee ground contact point
    model.addGroundContactPoint(bodyID, withLegSigns<T>(_kneeLocation, legID), false);

    // Knee Joint
    bodyID++;
    Mat6<T> xtreeKnee = createSXform(I3, withLegSigns<T>(_kneeLocation, legID));
    Mat6<T> xtreeKneeRotor = createSXform(I3, withLegSigns<T>(_kneeRotorLocation, legID));
    model.addBody(orient_inertia(_kneeInertia, sideSign), orient_inertia(_kneeRotorInertia, sideSign),
                  _kneeGearRatio, bodyID - 1, _knee_joint_type,
                  _knee_axis, xtreeKnee, xtreeKneeRotor);

    model.addGroundContactPoint(bodyID, withLegSigns<T>(_footLocation, legID), true);

    sideSign *= -1;
  }
  Vec3<T> g(0, 0, gravity);

  model.setGravity(g);

  return model;
}

/*!
 * flip inertia depending on leg
 */
template <typename T>
SpatialInertia<T> orient_inertia(SpatialInertia<T> unoriented, T sideSign)
{
  return sideSign > 0 ? unoriented : unoriented.flipAlongAxis(CoordinateAxis::Y);
}

/*!
 * Flip signs of elements of a vector V depending on which leg it belongs to
 */
template <typename T, typename T2>
Vec3<T> withLegSigns(const Eigen::MatrixBase<T2> &v, int legID)
{
  static_assert(T2::ColsAtCompileTime == 1 && T2::RowsAtCompileTime == 3,
                "Must have 3x1 matrix");
  switch (legID)
  {
  case 0:
    return Vec3<T>(v[0], -v[1], v[2]);
  case 1:
    return Vec3<T>(v[0], v[1], v[2]);
  default:
    throw std::runtime_error("Invalid leg id!");
  }
}

/*!
 * Build actuator models for a leg
 */
template <typename T>
std::vector<ActuatorModel<T> *> PatBiped<T>::buildActuatorModels() const
{
  std::vector<ActuatorModel<T> *> models;
  for (size_t i(0); i < 4; ++i)
  {
    ActuatorModel<T> *abadAct = new PatBipedActuatorModel<T>(_abadGearRatio, _abdMotorKT,
                                                             _abdMotorR, _batteryV,
                                                             _abdJointDamping, _abdJointDryFriction,
                                                            _abdMotorTauMax);

    ActuatorModel<T> *hipAct = new PatBipedActuatorModel<T>(_hipGearRatio, _motorKT,
                                                            _motorR, _batteryV, _jointDamping, _jointDryFriction, _motorTauMax);
    ActuatorModel<T> *kneeAct = new PatBipedActuatorModel<T>(_kneeGearRatio, _motorKT,
                                                             _motorR, _batteryV, _jointDamping, _jointDryFriction, _motorTauMax);

    models.push_back(abadAct);
    models.push_back(hipAct);
    models.push_back(kneeAct);
  }
  return models;
}

template class PatBiped<double>;
template class PatBiped<float>;
