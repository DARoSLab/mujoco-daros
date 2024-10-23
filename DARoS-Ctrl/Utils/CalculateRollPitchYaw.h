#ifndef CALCULATE_ROLL_PITCH_YAW
#define CALCULATE_ROLL_PITCH_YAW

/*! @file CalcLeastSquaresPlane.h
 *
 */

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#include "cppTypes.h"
#include "pseudoInverse.h"

Vec3<float> calculateLeastSquaresPlane(Vec4<float> x, Vec4<float> y, Vec4<float> z) {
  //Make A a 4x3 matrix in the form:
  // [[1, x(0), y(0)],
  //  [1, x(1), y(1)],
  //  [1, x(2), y(2)],
  //  [1, x(3), y(3)]]

  DMat<float> A = DMat<float>::Constant(4,3,0.0);

  for(int row = 0; row < 4; row++){
    A(row, 0) = 1.0;
    A(row, 1) = x(row);
    A(row, 2) = y(row);
  }

  DMat<float> invA = DMat<float>::Constant(4,3,0);
  pseudoInverse<float>(A.transpose()*A, 0.001, invA);

  return invA*A.transpose()*z;
}

//take in x,y,z for all 4 feet
Vec3<float> calculateRollPitchYaw(Vec4<float> x, Vec4<float> y, Vec4<float> z, float yaw){
  //create a matrix of the form:
  //[[cos(yaw), -sin(yaw), 0],
  // [sin(yaw),  cos(yaw), 0],
  // [0,         0,        1]]
  Mat3<float> R_yaw = Mat3<float>::Constant(0.0);
  R_yaw << cos(yaw), -sin(yaw), 0.0,
           sin(yaw),  cos(yaw), 0.0,
           0.0,       0.0,      1.0;

  //call calcLeast
  Vec3<float> coefficients = calculateLeastSquaresPlane(x,y,z);

  //calculate the roll, pitch, yaw adjustments
  Vec3<float> coefficients_yaw = Vec3<float>::Constant(0.0); 
  coefficients_yaw << coefficients(0), (coefficients.tail(2).transpose() * R_yaw.block(0,0,2,2)).transpose();

  float pitch = atan2(coefficients_yaw(1), 1);
  float roll = atan2(coefficients_yaw(2), 1);

  Vec3<float> rollPitchYaw = Vec3<float>::Constant(0.0);
  rollPitchYaw << roll, pitch, coefficients_yaw(0);
  return rollPitchYaw;
}

#endif
