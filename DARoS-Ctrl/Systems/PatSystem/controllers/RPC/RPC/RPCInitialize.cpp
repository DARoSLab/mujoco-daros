//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RPCInitialize.cpp
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
// RPCINITIALIZE
//     [X_INITIAL,X_REF_INITIAL,U_REF_INITIAL,P_FOOT_REF] = RPCINITIALIZE(IN1,IN2,IN3,IN4,IN5,IN6,IN7,DT,M,IN10,IN11)
// Arguments    : const double in1[12]
//                const double in2[12]
//                const double in3[4]
//                const double in4[36]
//                const double in5[12]
//                const double in6[4]
//                const double in7[12]
//                double dt
//                double m
//                const double in10[3]
//                const double in11[3]
//                double X_initial[36]
//                double x_ref_initial[12]
//                double u_ref_initial[24]
//                double p_foot_ref[12]
// Return Type  : void
//
void RPCInitialize(const double in1[12], const double in2[12], const double in3
                   [4], const double in4[36], const double in5[12], const double
                   in6[4], const double in7[12], double dt, double m, const
                   double in10[3], const double in11[3], double X_initial[36],
                   double x_ref_initial[12], double u_ref_initial[24], double
                   p_foot_ref[12])
{
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t8;
  double t10;
  double t11;
  double t12;
  double t14;
  double t16;
  double t18;
  double t19;
  double t20;
  double t21;
  double t22;
  double t23;
  double t47;
  double t24;
  double t25;
  double t50;
  double t26;
  double t27;
  double t53;
  double t28;
  double t29;
  double t56;
  double t30;
  double t31;
  double t60;
  double t32;
  double t33;
  double t61;
  double t34;
  double t35;
  double t62;
  double t36;
  double t37;
  double t63;
  double t38;
  double t39;
  double t49;
  double t40;
  double t41;
  double t52;
  double t42;
  double t43;
  double t55;
  double t44;
  double t45;
  double t58;
  double t46;
  double t59;
  double t69;
  double t91;
  double t96;
  double t101;
  double t102;
  double t104;
  double t108;
  double t109;
  double t110;
  double t118;
  double t119;

  //     This function was generated by the Symbolic Math Toolbox version 7.1.
  //     05-Mar-2020 07:53:56
  t2 = dt * dt;
  t3 = 1.0 / m;
  t4 = in3[0] * in3[0];
  t5 = in3[1] * in3[1];
  t6 = in3[2] * in3[2];
  t7 = in3[3] * in3[3];
  t8 = t4 * t4;
  t10 = dt * in1[6] * 0.5;
  t11 = t5 * t5;
  t12 = dt * in1[7] * 0.5;
  t14 = t6 * t6;
  t16 = t7 * t7;
  t18 = std::cos(in1[3]);
  t19 = std::sin(in1[5]);
  t20 = std::cos(in1[5]);
  t21 = std::sin(in1[3]);
  t22 = std::sin(in1[4]);
  t23 = in5[0] * (in6[0] - 1.0);
  t47 = in4[12] * in6[0];
  t24 = (((t10 + t23) - t47) + in1[0]) - in7[0];
  t25 = in5[3] * (in6[1] - 1.0);
  t50 = in4[18] * in6[1];
  t26 = (((t10 + t25) - t50) + in1[0]) - in7[3];
  t27 = in5[6] * (in6[2] - 1.0);
  t53 = in4[24] * in6[2];
  t28 = (((t10 + t27) - t53) + in1[0]) - in7[6];
  t29 = in5[9] * (in6[3] - 1.0);
  t56 = in4[30] * in6[3];
  t30 = (((t10 + t29) - t56) + in1[0]) - in7[9];
  t31 = in5[2] * (in6[0] - 1.0);
  t60 = in4[14] * in6[0];
  t32 = (t31 - t60) + in1[2];
  t33 = in5[5] * (in6[1] - 1.0);
  t61 = in4[20] * in6[1];
  t34 = (t33 - t61) + in1[2];
  t35 = in5[8] * (in6[2] - 1.0);
  t62 = in4[26] * in6[2];
  t36 = (t35 - t62) + in1[2];
  t37 = in5[11] * (in6[3] - 1.0);
  t63 = in4[32] * in6[3];
  t38 = (t37 - t63) + in1[2];
  t39 = in5[1] * (in6[0] - 1.0);
  t49 = in4[13] * in6[0];
  t40 = (((t12 + t39) - t49) + in1[1]) - in7[1];
  t41 = in5[4] * (in6[1] - 1.0);
  t52 = in4[19] * in6[1];
  t42 = (((t12 + t41) - t52) + in1[1]) - in7[4];
  t43 = in5[7] * (in6[2] - 1.0);
  t55 = in4[25] * in6[2];
  t44 = (((t12 + t43) - t55) + in1[1]) - in7[7];
  t45 = in5[10] * (in6[3] - 1.0);
  t58 = in4[31] * in6[3];
  t46 = (((t12 + t45) - t58) + in1[1]) - in7[10];
  t59 = ((((((in4[16] * t8 * t24 + in4[22] * t11 * t26) + in4[28] * t14 * t28) +
            in4[34] * t16 * t30) - in4[15] * t8 * t40) - in4[21] * t11 * t42) -
         in4[27] * t14 * t44) - in4[33] * t16 * t46;
  t12 = ((((((in4[17] * t8 * t24 + in4[23] * t11 * t26) + in4[29] * t14 * t28) +
            in4[35] * t16 * t30) - in4[15] * t8 * t32) - in4[21] * t11 * t34) -
         in4[27] * t14 * t36) - in4[33] * t16 * t38;
  t69 = std::cos(in1[4]);
  t10 = ((((((in4[16] * t8 * t32 + in4[22] * t11 * t34) + in4[28] * t14 * t36) +
            in4[34] * t16 * t38) - in4[17] * t8 * t40) - in4[23] * t11 * t42) -
         in4[29] * t14 * t44) - in4[35] * t16 * t46;
  t91 = ((in4[15] * t4 + in4[21] * t5) + in4[27] * t6) + in4[33] * t7;
  t96 = ((in4[16] * t4 + in4[22] * t5) + in4[28] * t6) + in4[34] * t7;
  t101 = ((in4[17] * t4 + in4[23] * t5) + in4[29] * t6) + in4[35] * t7;
  t102 = 1.0 / in10[0];
  t104 = t19 * t21 - t18 * t20 * t22;
  t108 = t12 * (t18 * t19 + t20 * t21 * t22);
  t109 = t20 * t69 * t10;
  t110 = 1.0 / in10[1];
  t118 = (t59 * (t20 * t21 + t18 * t19 * t22) + t19 * t69 * t10) - t12 * (t18 *
    t20 - t19 * t21 * t22);
  t119 = 1.0 / in10[2];
  t10 = (t18 * t59 * t69 + t21 * t12 * t69) - t22 * t10;
  t12 = in4[15] * in3[0];
  t8 = in4[16] * in3[0];
  t11 = in4[17] * in3[0];
  t14 = in4[21] * in3[1];
  t16 = in4[22] * in3[1];
  t4 = in4[23] * in3[1];
  t5 = in4[27] * in3[2];
  t6 = in4[28] * in3[2];
  t7 = in4[29] * in3[2];
  t69 = in4[33] * in3[3];
  t18 = in4[34] * in3[3];
  t21 = in4[35] * in3[3];
  X_initial[0] = ((in1[0] + in11[0] * t2 * 0.5) + dt * in1[6]) + t2 * t3 * t91 *
    0.5;
  X_initial[1] = ((in1[1] + in11[1] * t2 * 0.5) + dt * in1[7]) + t2 * t3 * t96 *
    0.5;
  X_initial[2] = ((in1[2] + in11[2] * t2 * 0.5) + dt * in1[8]) + t2 * t3 * t101 *
    0.5;
  X_initial[3] = (in1[3] + dt * in1[9]) + t2 * t102 * ((t108 + t109) - t59 *
    t104) * 0.5;
  X_initial[4] = (in1[4] + dt * in1[10]) - t2 * t110 * t118 * 0.5;
  X_initial[5] = (in1[5] + dt * in1[11]) - t2 * t119 * t10 * 0.5;
  X_initial[6] = (in1[6] + dt * in11[0]) + dt * t3 * t91;
  X_initial[7] = (in1[7] + dt * in11[1]) + dt * t3 * t96;
  X_initial[8] = (in1[8] + dt * in11[2]) + dt * t3 * t101;
  X_initial[9] = in1[9] + dt * t102 * ((t108 + t109) - t59 * t104);
  X_initial[10] = in1[10] - dt * t110 * t118;
  X_initial[11] = in1[11] - dt * t119 * t10;
  X_initial[12] = -in3[0] * t24;
  X_initial[13] = -in3[0] * t40;
  X_initial[14] = -in3[0] * t32;
  X_initial[15] = t12;
  X_initial[16] = t8;
  X_initial[17] = t11;
  X_initial[18] = -in3[1] * t26;
  X_initial[19] = -in3[1] * t42;
  X_initial[20] = -in3[1] * t34;
  X_initial[21] = t14;
  X_initial[22] = t16;
  X_initial[23] = t4;
  X_initial[24] = -in3[2] * t28;
  X_initial[25] = -in3[2] * t44;
  X_initial[26] = -in3[2] * t36;
  X_initial[27] = t5;
  X_initial[28] = t6;
  X_initial[29] = t7;
  X_initial[30] = -in3[3] * t30;
  X_initial[31] = -in3[3] * t46;
  X_initial[32] = -in3[3] * t38;
  X_initial[33] = t69;
  X_initial[34] = t18;
  X_initial[35] = t21;
  x_ref_initial[0] = in4[0] + in2[0];
  x_ref_initial[1] = in4[1] + in2[1];
  x_ref_initial[2] = in4[2] + in2[2];
  x_ref_initial[3] = in4[3] + in2[3];
  x_ref_initial[4] = in4[4] + in2[4];
  x_ref_initial[5] = in4[5] + in2[5];
  x_ref_initial[6] = (in4[6] - t19 * in2[7]) + t20 * in2[6];
  x_ref_initial[7] = (in4[7] + t19 * in2[6]) + t20 * in2[7];
  x_ref_initial[8] = in4[8] + in2[8];
  x_ref_initial[9] = in4[9] + in2[9];
  x_ref_initial[10] = in4[10] + in2[10];
  x_ref_initial[11] = in4[11] + in2[11];
  u_ref_initial[0] = -in3[0] * t24;
  u_ref_initial[1] = -in3[0] * t40;
  u_ref_initial[2] = -in3[0] * t32;
  u_ref_initial[3] = t12;
  u_ref_initial[4] = t8;
  u_ref_initial[5] = t11;
  u_ref_initial[6] = -in3[1] * t26;
  u_ref_initial[7] = -in3[1] * t42;
  u_ref_initial[8] = -in3[1] * t34;
  u_ref_initial[9] = t14;
  u_ref_initial[10] = t16;
  u_ref_initial[11] = t4;
  u_ref_initial[12] = -in3[2] * t28;
  u_ref_initial[13] = -in3[2] * t44;
  u_ref_initial[14] = -in3[2] * t36;
  u_ref_initial[15] = t5;
  u_ref_initial[16] = t6;
  u_ref_initial[17] = t7;
  u_ref_initial[18] = -in3[3] * t30;
  u_ref_initial[19] = -in3[3] * t46;
  u_ref_initial[20] = -in3[3] * t38;
  u_ref_initial[21] = t69;
  u_ref_initial[22] = t18;
  u_ref_initial[23] = t21;
  p_foot_ref[0] = -t23 + t47;
  p_foot_ref[1] = -t39 + t49;
  p_foot_ref[2] = -t31 + t60;
  p_foot_ref[3] = -t25 + t50;
  p_foot_ref[4] = -t41 + t52;
  p_foot_ref[5] = -t33 + t61;
  p_foot_ref[6] = -t27 + t53;
  p_foot_ref[7] = -t43 + t55;
  p_foot_ref[8] = -t35 + t62;
  p_foot_ref[9] = -t29 + t56;
  p_foot_ref[10] = -t45 + t58;
  p_foot_ref[11] = -t37 + t63;
}

//
// File trailer for RPCInitialize.cpp
//
// [EOF]
//
