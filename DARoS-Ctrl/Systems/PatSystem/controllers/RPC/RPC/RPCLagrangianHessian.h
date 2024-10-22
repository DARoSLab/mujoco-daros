//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RPCLagrangianHessian.h
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 05-Mar-2020 07:55:47
//
#ifndef RPCLAGRANGIANHESSIAN_H
#define RPCLAGRANGIANHESSIAN_H

// Include Files
#include <cmath>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

// Function Declarations
extern void RPCLagrangianHessian(const double in1[12], const double in2[24],
  const double in3[4], const double in4[12], const double in5[24], double dt,
  double obj_factor, const double in8[44], const double in9[3], double
  lagrangian_hessian_nz[135]);

#endif

//
// File trailer for RPCLagrangianHessian.h
//
// [EOF]
//
