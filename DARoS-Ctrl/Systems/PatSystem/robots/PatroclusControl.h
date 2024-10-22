/*! @file MiniCheetahVision.h
 *  @brief Utility function to build a Mini Cheetah Pat object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_PAT_CONTROL_H
#define PROJECT_PAT_CONTROL_H

#include <FloatingBaseModel.h>
#include "PatBiped.h"

/*!
 * Generate a Pat model of Mini Cheetah
 */
template <typename T>
bool buildPatroclusControl(PatBiped<T> *pat)
{
    pat->_robotType = RobotType::PATROCLUS;

    // pat->_x_offset = 0.041;
    pat->_x_offset = 0;
    pat->_bodyMass = 1.745; //5.70984; //5.7;
    // pat->_bodyLength = 0.185 * 2; // For two leg stance
    pat->_bodyLength = 0.04 * 2;
    pat->_bodyWidth = 0.045 * 2;
    pat->_bodyHeight = 0.15 * 2;
    pat->_abadGearRatio = 6;
    pat->_hipGearRatio = 9;
    pat->_kneeGearRatio = 14.49;
    pat->_abadLinkX_offset = -0.0475;
    pat->_abadLinkZ_offset = -0.06127;
    pat->_abadLinkLength = 0.059;
    pat->_hipLinkY_offset = 0.01525;
    pat->_hipLinkLength = 0.2078;
    // pat->_kneeLinkLength = 0.175;
    // pat->_kneeLinkLength = 0.20;
    pat->_kneeLinkLength = 0.205;//0.23; //0.205;
    pat->_kneeLinkY_offset = 0.05129;
    pat->_kneeLinkX_offset = 0.01741;
    pat->_maxLegLength = 0.409;

    pat->_motorTauMax = 2.f; //3.f;
    pat->_batteryV = 24;
    pat->_motorKT = 0.091; //.05; // this is flux linkage * pole pairs
    pat->_motorR = 0.173;
    pat->_jointDamping = .01;
    pat->_jointDryFriction = .2;

    pat->_abdMotorTauMax = 1.5f; //3.f;
    pat->_abdMotorKT = 0.068; //.05; // this is flux linkage * pole pairs
    pat->_abdMotorR = 0.262;
    pat->_abdJointDamping = .01;
    pat->_abdJointDryFriction = .2;
    // pat->_jointDamping = .0;
    // pat->_jointDryFriction = .0;

    // rotor inertia if the rotor is oriented so it spins around the z-axis
    Mat3<T> rotorRotationalInertiaZ;
    rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
    rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

    Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
    Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
    Mat3<T> rotorRotationalInertiaX =
        RY * rotorRotationalInertiaZ * RY.transpose();
    Mat3<T> rotorRotationalInertiaY =
        RX * rotorRotationalInertiaZ * RX.transpose();

    // spatial inertias
    Mat3<T> abadRotationalInertia;
    abadRotationalInertia << 439, 0, 0, 0, 759, 0, 0, 0, 498;
    abadRotationalInertia = abadRotationalInertia * 1e-6;
    Vec3<T> abadCOM(0.05, 0, 0); // LEFT
    // SpatialInertia<T> abadInertia(0.54, abadCOM, abadRotationalInertia);
    SpatialInertia<T> abadInertia(0.54, abadCOM, abadRotationalInertia);

    Mat3<T> hipRotationalInertia;
    hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
    hipRotationalInertia = hipRotationalInertia * 1e-6;
    Vec3<T> hipCOM(0, 0.016, -0.02);
    SpatialInertia<T> hipInertia(0.684, hipCOM, hipRotationalInertia);

    Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
    kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
    kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
    kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
    Vec3<T> kneeCOM(0, 0, -0.061);
    SpatialInertia<T> kneeInertia(0.064, kneeCOM, kneeRotationalInertia);

    Vec3<T> rotorCOM(0, 0, 0);
    SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
    SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

    Mat3<T> bodyRotationalInertia;
    bodyRotationalInertia << 0.03286, 0, 0, 0, 0.03365, 0, 0, 0, 0.00916;
    Vec3<T> bodyCOM(0, 0, -0.1);
    SpatialInertia<T> bodyInertia(pat->_bodyMass, bodyCOM,
                                  bodyRotationalInertia);

    pat->_abadInertia = abadInertia;
    pat->_hipInertia = hipInertia;
    pat->_kneeInertia = kneeInertia;
    pat->_abadRotorInertia = rotorInertiaX;
    pat->_hipRotorInertia = rotorInertiaY;
    pat->_kneeRotorInertia = rotorInertiaY;
    pat->_bodyInertia = bodyInertia;

    // locations
    pat->_abadRotorLocation = Vec3<T>(pat->_abadLinkX_offset, (pat->_bodyWidth / 2), pat->_abadLinkZ_offset);
    pat->_abadLocation = pat->_abadRotorLocation;
    pat->_hipLocation = Vec3<T>(pat->_abadLinkLength, pat->_hipLinkY_offset, 0);
    pat->_hipRotorLocation = pat->_hipLocation;
    pat->_kneeLocation = Vec3<T>(pat->_kneeLinkX_offset, pat->_kneeLinkY_offset, -pat->_hipLinkLength);
    pat->_kneeRotorLocation = Vec3<T>(0, 0, 0);
    pat->_footLocation = Vec3<T>(0, 0, -pat->_kneeLinkLength);

    // joint and rotor axes (they happen to match here but they could not on a different robot)
    pat->_abad_axis = CoordinateAxis::X;
    pat->_hip_axis = CoordinateAxis::Y;
    pat->_knee_axis = CoordinateAxis::Y;

    // defining joint types
    pat->_abad_joint_type = JointType::Revolute;
    pat->_hip_joint_type = JointType::Revolute;
    pat->_knee_joint_type = JointType::Revolute;

    // 2: stand up
    pat->_x_initial.bodyOrientation = rotationMatrixToQuaternion(
        ori::coordinateRotation(CoordinateAxis::Z, 0.0).cast<T>());
    // pat->_x_initial.bodyOrientation = rotationMatrixToQuaternion(
    // ori::coordinateRotation(CoordinateAxis::X, 0.5).cast<T>());
    pat->_x_initial.bodyPosition.setZero();
    pat->_x_initial.bodyVelocity.setZero();
    // pat->_x_initial.bodyVelocity[0] = 3.5;
    pat->_x_initial.bodyVelocity[0] = 0.0;
    pat->_x_initial.q = DVec<T>::Zero(pat_biped::num_act_joints);
    pat->_x_initial.qd = DVec<T>::Zero(pat_biped::num_act_joints);


    // pat->_x_initial.bodyPosition[2] = 0.45; //best
    pat->_x_initial.bodyPosition[2] = 0.42;
    // pat->_x_initial.bodyPosition[2] = 0.50; //rmp
    //Knee Backwards
    // pat->_x_initial.q[0] = 0.3;
    // pat->_x_initial.q[1] = 0.55;
    // pat->_x_initial.q[2] = -0.95;
    // pat->_x_initial.q[3] = -0.16;
    // pat->_x_initial.q[4] = 0.55;
    // pat->_x_initial.q[5] = -0.95;


    //Knee Forwards RL
    // pat->_x_initial.q[0] = 0.3;
    // pat->_x_initial.q[1] = -0.32;
    // pat->_x_initial.q[2] = 0.83;
    //
    // pat->_x_initial.q[3] = -0.16;
    // pat->_x_initial.q[4] = -0.29;
    // pat->_x_initial.q[5] = 0.81;

    //Knee Forwards TVR
    pat->_x_initial.q[0] = 0.16;
    pat->_x_initial.q[1] = -0.46;
    pat->_x_initial.q[2] = 1.2;

    pat->_x_initial.q[3] = -0.16;
    pat->_x_initial.q[4] = -0.46;
    pat->_x_initial.q[5] = 1.2;

    //Knee backwards TVR
    // pat->_x_initial.q[0] = 0.16;
    // pat->_x_initial.q[1] = 0.55;
    // pat->_x_initial.q[2] = -0.95;
    //
    // pat->_x_initial.q[3] = -0.3;
    // pat->_x_initial.q[4] = 0.55;
    // pat->_x_initial.q[5] = -0.95;

    // Mini Cheetah "home" posture - the base posture we return to in training
    pat->_x_home.bodyOrientation = rotationMatrixToQuaternion(
        ori::coordinateRotation(CoordinateAxis::Z, 0.0).cast<T>());
    pat->_x_home.bodyPosition.setZero();
    pat->_x_home.bodyVelocity.setZero();
    pat->_x_home.q = DVec<T>::Zero(pat_biped::num_act_joints);
    pat->_x_home.qd = DVec<T>::Zero(pat_biped::num_act_joints);
    pat->_x_home.bodyPosition[2] = 0.3;
    pat->_x_home.q[0] = -0.8;
    pat->_x_home.q[1] = 1.635;
    pat->_x_home.q[2] = 1.635;

    pat->_x_home.q[3] = -0.8;
    pat->_x_home.q[4] = 1.635;
    pat->_x_home.q[5] = 1.635;

    return true;
}

#endif // PROJECT_PATROCLUS_H
