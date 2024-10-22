/*! @file PatBipedForwardKinematics.h
*   @brief This file defines functions which calculate the position of Pat's legs based on joint positions.
*
*/

#ifndef PAT_FORWARD_KINEMATICS
#define PAT_FORWARD_KINEMATICS

#include "cppTypes.h"
#include "math.h"
#include <utils/Utilities/Mathutilities.h>
#include <robots/PatBiped.h>

namespace PatKinematics
{
    /*!
    * Orients vector based on sideSign
    */
    template <typename T>
    Vec3<T> orient_vector(Vec3<T> vector, T sideSign)
    {
        // flip y by sidesign
        vector(1) *= sideSign;
        return vector;
    }

    template <typename T>
    Vec3<T> coordinate_axis_to_vec(CoordinateAxis axis)
    {
        Vec3<T> axis_vec;
        switch (axis)
        {
        case CoordinateAxis::X:
            axis_vec << 1, 0, 0;
            break;
        case CoordinateAxis::Y:
            axis_vec << 0, 1, 0;
            break;
        case CoordinateAxis::Z:
            axis_vec << 0, 0, 1;
            break;
        }

        return axis_vec;
    }

    /*!
    * Calculates abad axis location in torso frame
    */
    template <typename T>
    Vec3<T> torso_abad_loc(PatBiped<T> &biped, Vec3<T> q, RotMat<T> &abad_rot, T sideSign)
    {
        // rotation is identity since it never rotates
        abad_rot << 1, 0, 0,
            0, 1, 0,
            0, 0, 1;

        return orient_vector(biped._abadLocation, sideSign);
    }

    /*!
    * Calculates hip axis location in abad frame
    */
    template <typename T>
    Vec3<T> abad_hip_loc(PatBiped<T> &biped, Vec3<T> q, RotMat<T> &hip_rot, T sideSign)
    {
        // generate a rotation matrix from axis and angle
        hip_rot = Eigen::AngleAxis<T>(q(0), coordinate_axis_to_vec<T>(biped._abad_axis)).toRotationMatrix();

        // rotate and return the hip location
        return hip_rot * biped._hipLocation;
    }

    /*!
    * Calculates knee axis location in hip frame
    */
    template <typename T>
    Vec3<T> hip_knee_loc(PatBiped<T> &biped, Vec3<T> q, RotMat<T> &knee_rot, T sideSign)
    {
        // generate a rotation matrix from axis and angle
        knee_rot = Eigen::AngleAxis<T>(q(1), coordinate_axis_to_vec<T>(biped._hip_axis)).toRotationMatrix();

        // rotate and return knee location
        return knee_rot * biped._kneeLocation;
    }

    /*!
    * Calculates foot location in knee frame
    */
    template <typename T>
    Vec3<T> knee_foot_loc(PatBiped<T> &biped, Vec3<T> q, RotMat<T> &foot_rot, T sideSign)
    {
        // generate a rotation matrix from axis and angle
        foot_rot = Eigen::AngleAxis<T>(q(2), coordinate_axis_to_vec<T>(biped._knee_axis)).toRotationMatrix();

        return foot_rot * biped._footLocation;
    }

    /*
    * Given biped and state vector and sideSign, sets rotation matrix
    * Returns knee position in the robot local frame
    */
    template <typename T>
    Vec3<T> fkKnee(PatBiped<T> &biped, Vec3<T> q, RotMat<T> &knee_rot, T sideSign)
    {
        // we define cumulative position and rotation variables and now will step through each joint adding its position and rotation
        Vec3<T> sum_pos;
        RotMat<T> sum_rot;
        RotMat<T> temp_rot;

        // initialize the sums with the torso position
        sum_pos = torso_abad_loc(biped, q, sum_rot, sideSign);

        // rotate the hip position in the abad frame by the rotation of the abad frame, and add the position to the global position of the previous link
        sum_pos += sum_rot * abad_hip_loc(biped, q, temp_rot, sideSign);
        sum_rot *= temp_rot;

        // do the same thing again
        sum_pos += sum_rot * hip_knee_loc(biped, q, temp_rot, sideSign);
        sum_rot *= temp_rot;

        knee_rot = sum_rot;
        return sum_pos;
    }

    /*
    * This is the same as the other fkKnee function but it recalculates sideSign, and doesn't set rotation
    */
    template <typename T>
    Vec3<T> fkKnee(PatBiped<T> &biped, Vec3<T> q, int leg)
    {
        RotMat<T> hip_rot;
        T sideSign = biped.getSideSign(leg);

        return fkKnee(biped, q, hip_rot, sideSign);
    }

    /*!
    * Calculates foot position in the robot torso frame
    */
    template <typename T>
    Vec3<T> fkFoot(PatBiped<T> &biped, Vec3<T> q, RotMat<T> &foot_rot, T sideSign)
    {
        // define cumulative position and rotation variables
        Vec3<T> sum_pos;
        RotMat<T> sum_rot;
        RotMat<T> temp_rot;

        // initialize with knee position and rotation
        sum_pos = fkKnee(biped, q, sum_rot, sideSign);

        // addrelative foot rotation converted to global position by global knee rotation
        sum_pos += sum_rot * knee_foot_loc(biped, q, temp_rot, sideSign);
        sum_rot *= foot_rot;

        foot_rot = sum_rot;
        return sum_pos;
    }

    /*!
    * Calculates foot position in the robot hip frame
    */
    template <typename T>
    Vec3<T> fkFoot(PatBiped<T> &biped, Vec3<T> q, int leg)
    {
        T sideSign = biped.getSideSign(leg);

        RotMat<T> temp_rot;
        return fkFoot(biped, q, temp_rot, sideSign);
    }

    template <typename T>
    Mat3<T> JFoot(PatBiped<T> &biped, Vec3<T> q, int leg)
    {
        Mat3<T> J;

        T sideSign = biped.getSideSign(leg);

        RotMat<T> knee_foot_rot;

        Vec3<T> knee_foot_loc = PatKinematics::knee_foot_loc(biped, q, knee_foot_rot, sideSign);

        RotMat<T> hip_knee_rot;

        Vec3<T> hip_foot_loc = knee_foot_loc + hip_knee_loc(biped, q, hip_knee_rot, sideSign);

        RotMat<T> hip_rot;

        Vec3<T> abad_foot_loc = hip_foot_loc + abad_hip_loc(biped, q, hip_rot, sideSign);

        RotMat<T> knee_rot = hip_rot * hip_knee_rot;

        J.col(0) = coordinate_axis_to_vec<T>(biped._abad_axis).cross(abad_foot_loc);

        J.col(1) = hip_rot * coordinate_axis_to_vec<T>(biped._hip_axis).cross(hip_foot_loc);

        J.col(2) = knee_rot * coordinate_axis_to_vec<T>(biped._knee_axis).cross(knee_foot_loc);

        return J;
    }
}
#endif