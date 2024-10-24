//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RPCCost.cpp
//
// MATLAB Coder version            : 3.2
// C/C++ source code generated on  : 05-Mar-2020 07:55:47
//

// Include Files
#include "RPCBounds.h"
#include "RPCConstraintJacobian.h"
#include "RPCConstraintJacobianFinal.h"
#include "RPCConstraintJacobianFinalSP.h"
#include "RPCConstraintJacobianSP.h"
#include "RPCConstraints.h"
#include "RPCConstraintsFinal.h"
#include "RPCCost.h"
#include "RPCCostGradient.h"
#include "RPCHeuristics.h"
#include "RPCInitialize.h"
#include "RPCLagrangianHessian.h"
#include "RPCLagrangianHessianFinal.h"
#include "RPCLagrangianHessianFinalSP.h"
#include "RPCLagrangianHessianSP.h"

// Function Definitions

//
// RPCCOST
//     J = RPCCOST(IN1,IN2,IN3,IN4,IN5,IN6)
// Arguments    : const double in1[12]
//                const double in2[24]
//                const double in3[12]
//                const double in4[24]
//                const double in5[12]
//                const double in6[24]
// Return Type  : double
//
double RPCCost(const double in1[12], const double in2[24], const double in3[12],
               const double in4[24], const double in5[12], const double in6[24])
{
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t9;
  double t10;
  double t11;
  double t12;
  double t13;
  double t14;
  double t15;
  double t16;
  double t17;
  double t18;
  double t19;
  double t20;
  double t21;
  double t22;
  double t23;
  double t24;
  double t25;
  double t26;
  double t27;
  double t28;
  double t29;
  double t30;
  double t31;
  double t32;
  double t33;
  double t34;
  double t35;
  double t36;
  double t37;

  //     This function was generated by the Symbolic Math Toolbox version 7.1.
  //     05-Mar-2020 07:53:59
  t2 = in2[3] - in4[3];
  t3 = in2[4] - in4[4];
  t4 = in2[5] - in4[5];
  t5 = in2[9] - in4[9];
  t6 = in2[10] - in4[10];
  t7 = in2[11] - in4[11];
  t8 = in2[15] - in4[15];
  t9 = in2[16] - in4[16];
  t10 = in2[17] - in4[17];
  t11 = in2[21] - in4[21];
  t12 = in2[22] - in4[22];
  t13 = in2[23] - in4[23];
  t14 = in2[0] - in4[0];
  t15 = in2[1] - in4[1];
  t16 = in2[2] - in4[2];
  t17 = in2[6] - in4[6];
  t18 = in2[7] - in4[7];
  t19 = in2[8] - in4[8];
  t20 = in2[12] - in4[12];
  t21 = in2[13] - in4[13];
  t22 = in2[14] - in4[14];
  t23 = in2[18] - in4[18];
  t24 = in2[19] - in4[19];
  t25 = in2[20] - in4[20];
  t26 = in1[0] - in3[0];
  t27 = in1[1] - in3[1];
  t28 = in1[2] - in3[2];
  t29 = in1[3] - in3[3];
  t30 = in1[4] - in3[4];
  t31 = in1[5] - in3[5];
  t32 = in1[6] - in3[6];
  t33 = in1[7] - in3[7];
  t34 = in1[8] - in3[8];
  t35 = in1[9] - in3[9];
  t36 = in1[10] - in3[10];
  t37 = in1[11] - in3[11];
  return ((((((((((((((((((((((((((((((((((in5[0] * (t26 * t26) + in5[1] * (t27 *
    t27)) + in5[2] * (t28 * t28)) + in5[3] * (t29 * t29)) + in5[4] * (t30 * t30))
    + in5[5] * (t31 * t31)) + in5[6] * (t32 * t32)) + in5[7] * (t33 * t33)) +
    in5[8] * (t34 * t34)) + in5[9] * (t35 * t35)) + in5[10] * (t36 * t36)) +
    in5[11] * (t37 * t37)) + in6[3] * (t2 * t2)) + in6[4] * (t3 * t3)) + in6[5] *
    (t4 * t4)) + in6[0] * (t14 * t14)) + in6[9] * (t5 * t5)) + in6[1] * (t15 *
    t15)) + in6[10] * (t6 * t6)) + in6[2] * (t16 * t16)) + in6[11] * (t7 * t7))
                       + in6[6] * (t17 * t17)) + in6[15] * (t8 * t8)) + in6[7] *
                     (t18 * t18)) + in6[16] * (t9 * t9)) + in6[8] * (t19 * t19))
                  + in6[17] * (t10 * t10)) + in6[12] * (t20 * t20)) + in6[21] *
                (t11 * t11)) + in6[13] * (t21 * t21)) + in6[22] * (t12 * t12)) +
             in6[14] * (t22 * t22)) + in6[23] * (t13 * t13)) + in6[18] * (t23 *
            t23)) + in6[19] * (t24 * t24)) + in6[20] * (t25 * t25);
}

//
// File trailer for RPCCost.cpp
//
// [EOF]
//