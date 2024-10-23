/*! @file Tello.h
 *  @brief Data structure containing parameters for quadruped robot
 */

#ifndef LIBBIOMIMETICS_Tello_H
#define LIBBIOMIMETICS_Tello_H

#include <vector>
#include <Dynamics/Robot.h>
#include <Dynamics/ActuatorModel.h>
#include <FBModel/FloatingBaseModel.h>
#include <Utilities/SpatialInertia.h>
#include "TelloActuatorModel.h"

#include <eigen3/Eigen/StdVector>


namespace tello {

  constexpr size_t num_act_joint = 18;
  constexpr size_t nDOF = num_act_joint+6;
  constexpr size_t num_limb = 4;
  constexpr size_t num_joint_group = num_limb;
  constexpr size_t num_leg = 2;
  constexpr size_t num_leg_joint = 5;
  constexpr size_t num_arm = 2;
  constexpr size_t num_arm_joint = 4;
  // right leg
  constexpr size_t R_hipRz    = 0;
  constexpr size_t R_hipRx    = 1;
  constexpr size_t R_hipRy    = 2;
  constexpr size_t R_knee     = 3;
  constexpr size_t R_ankleRy  = 4;

  constexpr size_t L_hipRz    = 5;
  constexpr size_t L_hipRx    = 6;
  constexpr size_t L_hipRy    = 7;
  constexpr size_t L_knee     = 8;
  constexpr size_t L_ankleRy  = 9;


  constexpr size_t shoulder_roll_R = 10;
  constexpr size_t shoulder_yaw_R = 11;
  constexpr size_t shoulder_pitch_R = 12;
  constexpr size_t elbow_R    = 13;

  constexpr size_t shoulder_roll_L = 14;
  constexpr size_t shoulder_yaw_L = 15;
  constexpr size_t shoulder_pitch_L = 16;
  constexpr size_t elbow_L    = 17;
}

//  Foot index
//  1 --- 2
//  |     |
//  3 --- 4
//   \   /
//    | |
//    heel

namespace tello_link { // numbering depends on order of how contact points are added
  constexpr size_t R_foot = 10; 
  constexpr size_t L_foot = 15;
}

//  Foot index (Right)
//  toe  1 --- 2
//       |     |
//  heel 1 --- 2

//  Foot index (Left)
//  toe  1 --- 2
//       |     |
//  heel 1 --- 2

namespace tello_contact{
  constexpr size_t R_heel = 8; 
  constexpr size_t R_toe = 9;

  constexpr size_t L_heel = 10; 
  constexpr size_t L_toe = 11;
  constexpr size_t num_foot_contact = 4;
}

/*!
 * Representation of a tello robot's physical properties.
 *
 * When viewed from the top, the tello's legs are:
 *
 * Leg
 * 1 0   RIGHT
 * 3 2   RIGHT
 * Arm
 *
 */
template <typename T>
class Tello : public Robot<T> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Tello(): Robot<T>() {
      this->NUM_FEET = 2;
      }
    virtual ~Tello() {}
    // depth, width, height
    Vec3<T> _waistDims, _torsoDims;

    T _torsoWidth;

    T _GearRatio;

    T _hipRzLinkLength, _hipRxLinkLength, _thighLength, _shankLength,
      _footToeLength, _footHeelLength, _footHeight, _maxLegLength;
    T _shoulderRxLinkLength, _shoulderRzLinkLength, _upperArmLength, _lowerArmLength, _maxArmLength;

    T _smallMotorKT, _smallMotorR, _largeMotorKT, _largeMotorR, _batteryV;
    T _smallMotorTauMax, _largeMotorTauMax;
    T _jointDamping, _jointDryFriction;
    T _smallMotorFluxLinkage, _smallMotorInductance, _smallMotorPoles;
    T _largeMotorFluxLinkage, _largeMotorInductance, _largeMotorPoles;

    SpatialInertia<T> _hipRzInertia, _hipRxInertia, _hipRyInertia, _kneeInertia, _ankleInertia,
      _hipRzRotorInertia,  _hipRxRotorInertia, _hipRyRotorInertia, _kneeRotorInertia, _ankleRotorInertia;
    SpatialInertia<T> _shoulderRyInertia, _shoulderRxInertia, _shoulderRzInertia, _elbowInertia,
      _shoulderRyRotorInertia, _shoulderRxRotorInertia,_shoulderRzRotorInertia, _elbowRotorInertia;
    SpatialInertia<T> _torsoInertia, _waistInertia;

    Vec3<T> _hipRzLocation, _hipRxLocation, _hipRyLocation, _kneeLocation, _ankleLocation,
      _hipRzRotorLocation, _hipRxRotorLocation, _hipRyRotorLocation, _kneeRotorLocation, _ankleRotorLocation;

    T _hipRzPitch,_hipRxPitch,_hipRyPitch;
    Vec3<T> _shoulderRxLocation, _shoulderRyLocation, _shoulderRzLocation, _elbowLocation,
      _shoulderRxRotorLocation, _shoulderRyRotorLocation, _shoulderRzRotorLocation, _elbowRotorLocation;

    virtual FloatingBaseModel<T> buildModel(T gravity = -9.81) const ;
    virtual std::vector<ActuatorModel<T>*> buildActuatorModels() const;
    virtual int numActJoint() const { return tello::num_act_joint; }
    virtual void getInitialState(FBModelState<T> & x0) const;

    static T getSideSign(int leg) {
      const T sideSigns[4] = { -1, 1, -1, 1};
      assert(leg >= 0 && leg < 4);
      return sideSigns[leg];
    }

  // virtual Vec3<T> getHipLocation(int leg) const {
  //   //0:right, 1:left
  //   //printf("leg: %d\n", leg);
  //   Vec3<T> pHip(
  //     _hipRzLocation(0),
  //     (leg == 0 ) ? -_torsoWidth/2. : _torsoWidth/2.,
  //     _hipRzLocation(2));
  //   return pHip;
  // }
};

#endif  // LIBBIOMIMETICS_Tello_H
