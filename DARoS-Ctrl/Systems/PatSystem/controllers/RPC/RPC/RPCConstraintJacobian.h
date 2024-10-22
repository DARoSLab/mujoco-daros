//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RPCConstraintJacobian.h
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 05-Mar-2020 07:55:47
//
#ifndef RPCCONSTRAINTJACOBIAN_H
#define RPCCONSTRAINTJACOBIAN_H

// Include Files
#include <cmath>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

// Function Declarations
extern void RPCConstraintJacobian(const double in1[12], const double in2[24],
  const double in3[4], const double in4[4], const double in5[4], double dt,
  double m, const double in8[3], const double in9[4], double
  constraint_jacobian_nz[300]);

#endif

//
// File trailer for RPCConstraintJacobian.h
//
// [EOF]
//
