/*! @file Tello.h
 *  @brief Utility function to build a robot object
 */

#ifndef Tello_H
#define Tello_H

#include <Utilities/spatial.h>
#include <Dynamics/FloatingBaseModel.h>
#include "Tello.h"
#include <cppTypes.h>

using namespace spatial;

template <typename T> bool setActuatorParams(Tello<T>* tello);

/*!
 * Generate a tello model of the Humanoid robot
 */
template <typename T>
bool setTelloRobotParams(Tello<T>* tello) {
  // Waist dimensions for box collisions ---------------------------
  tello->_torsoWidth = 0.2;
  tello->_waistDims << 0.1, 0.164, 0.1;
  tello->_torsoDims << 0.12, 0.14, 0.16;
  
  // tello->_totalMass = 18.811378;  //Simon added, read from URDF
  // Actuator (rotor inertia) setup ----------------------------------------------
  setActuatorParams(tello);

  return true;
}

template <typename T> bool setActuatorParams(Tello<T>* tello){
  // Need to verify damping and friction
  tello->_GearRatio = 10;
  tello->_batteryV = 60;
  tello->_smallMotorKT = 0.1567;
  tello->_smallMotorR = 0.43;
  tello->_smallMotorFluxLinkage = 0.005;
  tello->_smallMotorInductance = 0.002*(10e-6) / 1.5;  //Simon modified this value to make robot jump higher
  tello->_smallMotorPoles = 21;
  tello->_largeMotorKT = 0.191;
  tello->_largeMotorR = 0.158;
  tello->_largeMotorFluxLinkage = 0.006;
  tello->_largeMotorInductance = 83*(10e-6) / 1.5;
  tello->_largeMotorPoles = 21;
  tello->_jointDamping = .1;
  tello->_jointDryFriction = 1;
  tello->_smallMotorTauMax = 200.;
  tello->_largeMotorTauMax = 200.;

  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> largeRotorRotationalInertiaZ;
  largeRotorRotationalInertiaZ << 55, 0, 0, 0, 55, 0, 0, 0, 55; // This is wrong number (These are actuator's inertia)
  largeRotorRotationalInertiaZ = 1e-6 * largeRotorRotationalInertiaZ;

  Mat3<T> smallRotorRotationalInertiaZ;
  smallRotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 33;
  smallRotorRotationalInertiaZ = 1e-6 * smallRotorRotationalInertiaZ;

  // todo check hip Rz and hip Rx rotor inertias which are no longer pure 90 deg rotations
  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  
  Mat3<T> smallRotorRotationalInertiaX =
      RY * smallRotorRotationalInertiaZ * RY.transpose();
  Mat3<T> smallRotorRotationalInertiaY =
      RX * smallRotorRotationalInertiaZ * RX.transpose();
  Mat3<T> largeRotorRotationalInertiaY =
      RX * largeRotorRotationalInertiaZ * RX.transpose();
  // spatial inertias - rotors
  Vec3<T> smallRotorCOM(0, 0, 0);
  SpatialInertia<T> smallRotorInertiaZ(0., smallRotorCOM, smallRotorRotationalInertiaZ);
  SpatialInertia<T> smallRotorInertiaX(0., smallRotorCOM, smallRotorRotationalInertiaX);
  SpatialInertia<T> smallRotorInertiaY(0., smallRotorCOM, smallRotorRotationalInertiaY);

  tello->_shoulderRyRotorInertia = smallRotorInertiaY;
  tello->_shoulderRxRotorInertia = smallRotorInertiaX;
  tello->_shoulderRzRotorInertia = smallRotorInertiaZ;
  tello->_elbowRotorInertia = smallRotorInertiaY;

  Vec3<T> largeRotorCOMy(0, -0.019, 0);
  SpatialInertia<T> largeRotorInertiaY(0., largeRotorCOMy, largeRotorRotationalInertiaY);

  return true;
}
#endif  
