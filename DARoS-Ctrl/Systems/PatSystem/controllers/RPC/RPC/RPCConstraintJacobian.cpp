//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RPCConstraintJacobian.cpp
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
// RPCCONSTRAINTJACOBIAN
//     CONSTRAINT_JACOBIAN_NZ = RPCCONSTRAINTJACOBIAN(IN1,IN2,IN3,IN4,IN5,DT,M,IN8,IN9)
// Arguments    : const double in1[12]
//                const double in2[24]
//                const double in3[4]
//                const double in4[4]
//                const double in5[4]
//                double dt
//                double m
//                const double in8[3]
//                const double in9[4]
//                double constraint_jacobian_nz[300]
// Return Type  : void
//
void RPCConstraintJacobian(const double in1[12], const double in2[24], const
  double in3[4], const double in4[4], const double in5[4], double dt, double m,
  const double in8[3], const double in9[4], double constraint_jacobian_nz[300])
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
  double t16;
  double t21;
  double t26;
  double t31;
  double t34;
  double t37;
  double t39;
  double t40;
  double t42;
  double t46;
  double t48;
  double t49;
  double t51;
  double t56;
  double t64;
  double t68;
  double t71;
  double t76;
  double t79;
  double t80;
  double t81;
  double t84;
  double t86;
  double t89;
  double t91;
  double t93;
  double t96;
  double t99;
  double t100;
  double t102;
  double t104;
  double t107;
  double t109;
  double t112;
  double t114;
  double t116;
  double t119;
  double t122;
  double t123;
  double t124;
  double t127;
  double t129;
  double t132;
  double t134;
  double t136;
  double t139;
  double t142;
  double t144;
  double t146;
  double t149;
  double t151;
  double t154;
  double t156;
  double t158;
  double t161;
  double t164;
  double t165;
  double t166;
  double t169;
  double t171;
  double t174;
  double t176;
  double t178;
  double t181;
  double t184;
  double t186;
  double t188;
  double t191;
  double t193;
  double t196;
  double t198;
  double t200;
  double t203;
  double t206;
  double t207;
  double t208;
  double t211;
  double t213;
  double t216;
  double t218;
  double t220;
  double t223;
  double t226;
  double t228;
  double t230;
  double t233;
  double t235;
  double t238;
  double t240;
  double t242;

  //     This function was generated by the Symbolic Math Toolbox version 7.1.
  //     05-Mar-2020 07:54:09
  t2 = in3[0] * in3[0];
  t3 = in3[1] * in3[1];
  t4 = in3[2] * in3[2];
  t5 = in3[3] * in3[3];
  t6 = std::sin(in1[3]);
  t7 = std::sin(in1[5]);
  t8 = std::cos(in1[3]);
  t9 = std::cos(in1[5]);
  t10 = std::sin(in1[4]);
  t11 = dt * dt;
  t16 = ((((((in2[3] * in2[1] * t2 + in2[9] * in2[7] * t3) + in2[15] * in2[13] *
             t4) + in2[21] * in2[19] * t5) - in2[4] * in2[0] * t2) - in2[10] *
          in2[6] * t3) - in2[16] * in2[12] * t4) - in2[22] * in2[18] * t5;
  t21 = ((((((in2[3] * in2[2] * t2 + in2[9] * in2[8] * t3) + in2[15] * in2[14] *
             t4) + in2[21] * in2[20] * t5) - in2[5] * in2[0] * t2) - in2[11] *
          in2[6] * t3) - in2[17] * in2[12] * t4) - in2[23] * in2[18] * t5;
  t26 = std::cos(in1[4]);
  t31 = 1.0 / in8[0];
  t34 = t7 * t8 + t6 * t9 * t10;
  t37 = t6 * t7 - t8 * t9 * t10;
  t39 = t16 * t34 + t21 * t37;
  t40 = 1.0 / in8[1];
  t42 = t8 * t9 - t6 * t7 * t10;
  t46 = t6 * t9 + t7 * t8 * t10;
  t48 = t16 * t42 + t21 * t46;
  t49 = 1.0 / in8[2];
  t51 = t6 * t16 * t26 - t8 * t21 * t26;
  t56 = ((((((in2[4] * in2[2] * t2 + in2[10] * in2[8] * t3) + in2[16] * in2[14] *
             t4) + in2[22] * in2[20] * t5) - in2[5] * in2[1] * t2) - in2[11] *
          in2[7] * t3) - in2[17] * in2[13] * t4) - in2[23] * in2[19] * t5;
  t64 = (t9 * t10 * t56 + t8 * t9 * t16 * t26) + t6 * t9 * t21 * t26;
  t68 = (t7 * t10 * t56 + t7 * t8 * t16 * t26) + t6 * t7 * t21 * t26;
  t71 = (t8 * t10 * t16 + t6 * t10 * t21) - t26 * t56;
  t76 = (t21 * t42 + t7 * t26 * t56) - t16 * t46;
  t79 = (t16 * t37 + t9 * t26 * t56) - t21 * t34;
  t80 = in2[5] * t2 * t34;
  t81 = in2[5] * t2 * t42;
  t84 = in2[4] * t8 + in2[5] * t6;
  t86 = in2[3] * t2 * t37 - in2[5] * t2 * t9 * t26;
  t89 = in2[3] * t2 * t46 + in2[5] * t2 * t7 * t26;
  t91 = in2[5] * t10 - in2[3] * t8 * t26;
  t93 = in2[3] * t2 * t34 - in2[4] * t2 * t9 * t26;
  t96 = in2[3] * t2 * t42 + in2[4] * t2 * t7 * t26;
  t99 = in2[4] * t10 + in2[3] * t6 * t26;
  t100 = 1.0 / m;
  t102 = in2[2] * t2 * t34 - in2[1] * t2 * t37;
  t104 = in2[2] * t2 * t42 - in2[1] * t2 * t46;
  t107 = in2[1] * t8 + in2[2] * t6;
  t109 = in2[0] * t2 * t37 - in2[2] * t2 * t9 * t26;
  t112 = in2[0] * t2 * t46 + in2[2] * t2 * t7 * t26;
  t114 = in2[2] * t10 - in2[0] * t8 * t26;
  t116 = in2[0] * t2 * t34 - in2[1] * t2 * t9 * t26;
  t119 = in2[0] * t2 * t42 + in2[1] * t2 * t7 * t26;
  t122 = in2[1] * t10 + in2[0] * t6 * t26;
  t123 = in2[11] * t3 * t34;
  t124 = in2[11] * t3 * t42;
  t127 = in2[10] * t8 + in2[11] * t6;
  t129 = in2[9] * t3 * t37 - in2[11] * t3 * t9 * t26;
  t132 = in2[9] * t3 * t46 + in2[11] * t3 * t7 * t26;
  t134 = in2[11] * t10 - in2[9] * t8 * t26;
  t136 = in2[9] * t3 * t34 - in2[10] * t3 * t9 * t26;
  t139 = in2[9] * t3 * t42 + in2[10] * t3 * t7 * t26;
  t142 = in2[10] * t10 + in2[9] * t6 * t26;
  t144 = in2[8] * t3 * t34 - in2[7] * t3 * t37;
  t146 = in2[8] * t3 * t42 - in2[7] * t3 * t46;
  t149 = in2[7] * t8 + in2[8] * t6;
  t151 = in2[6] * t3 * t37 - in2[8] * t3 * t9 * t26;
  t154 = in2[6] * t3 * t46 + in2[8] * t3 * t7 * t26;
  t156 = in2[8] * t10 - in2[6] * t8 * t26;
  t158 = in2[6] * t3 * t34 - in2[7] * t3 * t9 * t26;
  t161 = in2[6] * t3 * t42 + in2[7] * t3 * t7 * t26;
  t164 = in2[7] * t10 + in2[6] * t6 * t26;
  t165 = in2[17] * t4 * t34;
  t166 = in2[17] * t4 * t42;
  t169 = in2[16] * t8 + in2[17] * t6;
  t171 = in2[15] * t4 * t37 - in2[17] * t4 * t9 * t26;
  t174 = in2[15] * t4 * t46 + in2[17] * t4 * t7 * t26;
  t176 = in2[17] * t10 - in2[15] * t8 * t26;
  t178 = in2[15] * t4 * t34 - in2[16] * t4 * t9 * t26;
  t181 = in2[15] * t4 * t42 + in2[16] * t4 * t7 * t26;
  t184 = in2[16] * t10 + in2[15] * t6 * t26;
  t186 = in2[14] * t4 * t34 - in2[13] * t4 * t37;
  t188 = in2[14] * t4 * t42 - in2[13] * t4 * t46;
  t191 = in2[13] * t8 + in2[14] * t6;
  t193 = in2[12] * t4 * t37 - in2[14] * t4 * t9 * t26;
  t196 = in2[12] * t4 * t46 + in2[14] * t4 * t7 * t26;
  t198 = in2[14] * t10 - in2[12] * t8 * t26;
  t200 = in2[12] * t4 * t34 - in2[13] * t4 * t9 * t26;
  t203 = in2[12] * t4 * t42 + in2[13] * t4 * t7 * t26;
  t206 = in2[13] * t10 + in2[12] * t6 * t26;
  t207 = in2[23] * t5 * t34;
  t208 = in2[23] * t5 * t42;
  t211 = in2[22] * t8 + in2[23] * t6;
  t213 = in2[21] * t5 * t37 - in2[23] * t5 * t9 * t26;
  t216 = in2[21] * t5 * t46 + in2[23] * t5 * t7 * t26;
  t218 = in2[23] * t10 - in2[21] * t8 * t26;
  t220 = in2[21] * t5 * t34 - in2[22] * t5 * t9 * t26;
  t223 = in2[21] * t5 * t42 + in2[22] * t5 * t7 * t26;
  t226 = in2[22] * t10 + in2[21] * t6 * t26;
  t228 = in2[20] * t5 * t34 - in2[19] * t5 * t37;
  t230 = in2[20] * t5 * t42 - in2[19] * t5 * t46;
  t233 = in2[19] * t8 + in2[20] * t6;
  t235 = in2[18] * t5 * t37 - in2[20] * t5 * t9 * t26;
  t238 = in2[18] * t5 * t46 + in2[20] * t5 * t7 * t26;
  t240 = in2[20] * t10 - in2[18] * t8 * t26;
  t242 = in2[18] * t5 * t34 - in2[19] * t5 * t9 * t26;
  t34 = in2[18] * t5 * t42 + in2[19] * t5 * t7 * t26;
  t16 = in2[19] * t10 + in2[18] * t6 * t26;
  t21 = in4[0] * in3[0];
  t56 = in4[1] * in3[1];
  t8 = in4[2] * in3[2];
  t9 = in4[3] * in3[3];
  constraint_jacobian_nz[0] = -1.0;
  constraint_jacobian_nz[1] = -in4[0] * in3[0];
  constraint_jacobian_nz[2] = -in4[1] * in3[1];
  constraint_jacobian_nz[3] = -in4[2] * in3[2];
  constraint_jacobian_nz[4] = -in4[3] * in3[3];
  constraint_jacobian_nz[5] = -1.0;
  constraint_jacobian_nz[6] = -in4[0] * in3[0];
  constraint_jacobian_nz[7] = -in4[1] * in3[1];
  constraint_jacobian_nz[8] = -in4[2] * in3[2];
  constraint_jacobian_nz[9] = -in4[3] * in3[3];
  constraint_jacobian_nz[10] = -1.0;
  constraint_jacobian_nz[11] = -in3[0] * in5[0];
  constraint_jacobian_nz[12] = -in3[1] * in5[1];
  constraint_jacobian_nz[13] = -in3[2] * in5[2];
  constraint_jacobian_nz[14] = -in3[3] * in5[3];
  constraint_jacobian_nz[15] = -in4[0] * in3[0];
  constraint_jacobian_nz[16] = -in4[1] * in3[1];
  constraint_jacobian_nz[17] = -in4[2] * in3[2];
  constraint_jacobian_nz[18] = -in4[3] * in3[3];
  constraint_jacobian_nz[19] = t11 * t31 * t39 * 0.5 - 1.0;
  constraint_jacobian_nz[20] = t11 * t40 * t48 * 0.5;
  constraint_jacobian_nz[21] = t11 * t49 * t51 * -0.5;
  constraint_jacobian_nz[22] = dt * t31 * t39;
  constraint_jacobian_nz[23] = dt * t40 * t48;
  constraint_jacobian_nz[24] = -dt * t49 * t51;
  constraint_jacobian_nz[25] = t11 * t31 * t64 * -0.5;
  constraint_jacobian_nz[26] = t11 * t40 * t68 * 0.5 - 1.0;
  constraint_jacobian_nz[27] = t11 * t49 * t71 * -0.5;
  constraint_jacobian_nz[28] = -dt * t31 * t64;
  constraint_jacobian_nz[29] = dt * t40 * t68;
  constraint_jacobian_nz[30] = -dt * t49 * t71;
  constraint_jacobian_nz[31] = t11 * t31 * t76 * -0.5;
  constraint_jacobian_nz[32] = t11 * t40 * t79 * -0.5;
  constraint_jacobian_nz[33] = -1.0;
  constraint_jacobian_nz[34] = -dt * t31 * t76;
  constraint_jacobian_nz[35] = -dt * t40 * t79;
  constraint_jacobian_nz[36] = -dt;
  constraint_jacobian_nz[37] = -1.0;
  constraint_jacobian_nz[38] = -dt;
  constraint_jacobian_nz[39] = -1.0;
  constraint_jacobian_nz[40] = -dt;
  constraint_jacobian_nz[41] = -1.0;
  constraint_jacobian_nz[42] = -dt;
  constraint_jacobian_nz[43] = -1.0;
  constraint_jacobian_nz[44] = -dt;
  constraint_jacobian_nz[45] = -1.0;
  constraint_jacobian_nz[46] = -dt;
  constraint_jacobian_nz[47] = -1.0;
  constraint_jacobian_nz[48] = t11 * t31 * (t80 - in2[4] * t2 * t37) * 0.5;
  constraint_jacobian_nz[49] = t11 * t40 * (t81 - in2[4] * t2 * t46) * 0.5;
  constraint_jacobian_nz[50] = t2 * t11 * t26 * t49 * t84 * -0.5;
  constraint_jacobian_nz[51] = dt * t31 * (t80 - in2[4] * t2 * t37);
  constraint_jacobian_nz[52] = dt * t40 * (t81 - in2[4] * t2 * t46);
  constraint_jacobian_nz[53] = -dt * t2 * t26 * t49 * t84;
  constraint_jacobian_nz[54] = -in4[0] * in3[0];
  constraint_jacobian_nz[55] = t11 * t31 * t86 * 0.5;
  constraint_jacobian_nz[56] = t11 * t40 * t89 * 0.5;
  constraint_jacobian_nz[57] = t2 * t11 * t49 * t91 * -0.5;
  constraint_jacobian_nz[58] = dt * t31 * t86;
  constraint_jacobian_nz[59] = dt * t40 * t89;
  constraint_jacobian_nz[60] = -dt * t2 * t49 * t91;
  constraint_jacobian_nz[61] = -in4[0] * in3[0];
  constraint_jacobian_nz[62] = t11 * t31 * t93 * -0.5;
  constraint_jacobian_nz[63] = t11 * t40 * t96 * -0.5;
  constraint_jacobian_nz[64] = t2 * t11 * t49 * t99 * 0.5;
  constraint_jacobian_nz[65] = -dt * t31 * t93;
  constraint_jacobian_nz[66] = -dt * t40 * t96;
  constraint_jacobian_nz[67] = dt * t2 * t49 * t99;
  constraint_jacobian_nz[68] = -in3[0] * in5[0];
  constraint_jacobian_nz[69] = -in4[0] * in3[0];
  constraint_jacobian_nz[70] = in3[0] * t11 * t100 * -0.5;
  constraint_jacobian_nz[71] = t11 * t31 * t102 * -0.5;
  constraint_jacobian_nz[72] = t11 * t40 * t104 * -0.5;
  constraint_jacobian_nz[73] = t2 * t11 * t26 * t49 * t107 * 0.5;
  constraint_jacobian_nz[74] = -in3[0] * dt * t100;
  constraint_jacobian_nz[75] = -dt * t31 * t102;
  constraint_jacobian_nz[76] = -dt * t40 * t104;
  constraint_jacobian_nz[77] = dt * t2 * t26 * t49 * t107;
  constraint_jacobian_nz[78] = in3[0];
  constraint_jacobian_nz[79] = -in3[0];
  constraint_jacobian_nz[80] = in3[0] * t11 * t100 * -0.5;
  constraint_jacobian_nz[81] = t11 * t31 * t109 * -0.5;
  constraint_jacobian_nz[82] = t11 * t40 * t112 * -0.5;
  constraint_jacobian_nz[83] = t2 * t11 * t49 * t114 * 0.5;
  constraint_jacobian_nz[84] = -in3[0] * dt * t100;
  constraint_jacobian_nz[85] = -dt * t31 * t109;
  constraint_jacobian_nz[86] = -dt * t40 * t112;
  constraint_jacobian_nz[87] = dt * t2 * t49 * t114;
  constraint_jacobian_nz[88] = in3[0];
  constraint_jacobian_nz[89] = -in3[0];
  constraint_jacobian_nz[90] = in3[0] * t11 * t100 * -0.5;
  constraint_jacobian_nz[91] = t11 * t31 * t116 * 0.5;
  constraint_jacobian_nz[92] = t11 * t40 * t119 * 0.5;
  constraint_jacobian_nz[93] = t2 * t11 * t49 * t122 * -0.5;
  constraint_jacobian_nz[94] = -in3[0] * dt * t100;
  constraint_jacobian_nz[95] = dt * t31 * t116;
  constraint_jacobian_nz[96] = dt * t40 * t119;
  constraint_jacobian_nz[97] = -dt * t2 * t49 * t122;
  constraint_jacobian_nz[98] = -in3[0] * in9[0];
  constraint_jacobian_nz[99] = -in3[0] * in9[0];
  constraint_jacobian_nz[100] = -in3[0] * in9[0];
  constraint_jacobian_nz[101] = -in3[0] * in9[0];
  constraint_jacobian_nz[102] = t11 * t31 * (t123 - in2[10] * t3 * t37) * 0.5;
  constraint_jacobian_nz[103] = t11 * t40 * (t124 - in2[10] * t3 * t46) * 0.5;
  constraint_jacobian_nz[104] = t3 * t11 * t26 * t49 * t127 * -0.5;
  constraint_jacobian_nz[105] = dt * t31 * (t123 - in2[10] * t3 * t37);
  constraint_jacobian_nz[106] = dt * t40 * (t124 - in2[10] * t3 * t46);
  constraint_jacobian_nz[107] = -dt * t3 * t26 * t49 * t127;
  constraint_jacobian_nz[108] = -in4[1] * in3[1];
  constraint_jacobian_nz[109] = t11 * t31 * t129 * 0.5;
  constraint_jacobian_nz[110] = t11 * t40 * t132 * 0.5;
  constraint_jacobian_nz[111] = t3 * t11 * t49 * t134 * -0.5;
  constraint_jacobian_nz[112] = dt * t31 * t129;
  constraint_jacobian_nz[113] = dt * t40 * t132;
  constraint_jacobian_nz[114] = -dt * t3 * t49 * t134;
  constraint_jacobian_nz[115] = -in4[1] * in3[1];
  constraint_jacobian_nz[116] = t11 * t31 * t136 * -0.5;
  constraint_jacobian_nz[117] = t11 * t40 * t139 * -0.5;
  constraint_jacobian_nz[118] = t3 * t11 * t49 * t142 * 0.5;
  constraint_jacobian_nz[119] = -dt * t31 * t136;
  constraint_jacobian_nz[120] = -dt * t40 * t139;
  constraint_jacobian_nz[121] = dt * t3 * t49 * t142;
  constraint_jacobian_nz[122] = -in3[1] * in5[1];
  constraint_jacobian_nz[123] = -in4[1] * in3[1];
  constraint_jacobian_nz[124] = in3[1] * t11 * t100 * -0.5;
  constraint_jacobian_nz[125] = t11 * t31 * t144 * -0.5;
  constraint_jacobian_nz[126] = t11 * t40 * t146 * -0.5;
  constraint_jacobian_nz[127] = t3 * t11 * t26 * t49 * t149 * 0.5;
  constraint_jacobian_nz[128] = -in3[1] * dt * t100;
  constraint_jacobian_nz[129] = -dt * t31 * t144;
  constraint_jacobian_nz[130] = -dt * t40 * t146;
  constraint_jacobian_nz[131] = dt * t3 * t26 * t49 * t149;
  constraint_jacobian_nz[132] = in3[1];
  constraint_jacobian_nz[133] = -in3[1];
  constraint_jacobian_nz[134] = in3[1] * t11 * t100 * -0.5;
  constraint_jacobian_nz[135] = t11 * t31 * t151 * -0.5;
  constraint_jacobian_nz[136] = t11 * t40 * t154 * -0.5;
  constraint_jacobian_nz[137] = t3 * t11 * t49 * t156 * 0.5;
  constraint_jacobian_nz[138] = -in3[1] * dt * t100;
  constraint_jacobian_nz[139] = -dt * t31 * t151;
  constraint_jacobian_nz[140] = -dt * t40 * t154;
  constraint_jacobian_nz[141] = dt * t3 * t49 * t156;
  constraint_jacobian_nz[142] = in3[1];
  constraint_jacobian_nz[143] = -in3[1];
  constraint_jacobian_nz[144] = in3[1] * t11 * t100 * -0.5;
  constraint_jacobian_nz[145] = t11 * t31 * t158 * 0.5;
  constraint_jacobian_nz[146] = t11 * t40 * t161 * 0.5;
  constraint_jacobian_nz[147] = t3 * t11 * t49 * t164 * -0.5;
  constraint_jacobian_nz[148] = -in3[1] * dt * t100;
  constraint_jacobian_nz[149] = dt * t31 * t158;
  constraint_jacobian_nz[150] = dt * t40 * t161;
  constraint_jacobian_nz[151] = -dt * t3 * t49 * t164;
  constraint_jacobian_nz[152] = -in3[1] * in9[1];
  constraint_jacobian_nz[153] = -in3[1] * in9[1];
  constraint_jacobian_nz[154] = -in3[1] * in9[1];
  constraint_jacobian_nz[155] = -in3[1] * in9[1];
  constraint_jacobian_nz[156] = t11 * t31 * (t165 - in2[16] * t4 * t37) * 0.5;
  constraint_jacobian_nz[157] = t11 * t40 * (t166 - in2[16] * t4 * t46) * 0.5;
  constraint_jacobian_nz[158] = t4 * t11 * t26 * t49 * t169 * -0.5;
  constraint_jacobian_nz[159] = dt * t31 * (t165 - in2[16] * t4 * t37);
  constraint_jacobian_nz[160] = dt * t40 * (t166 - in2[16] * t4 * t46);
  constraint_jacobian_nz[161] = -dt * t4 * t26 * t49 * t169;
  constraint_jacobian_nz[162] = -in4[2] * in3[2];
  constraint_jacobian_nz[163] = t11 * t31 * t171 * 0.5;
  constraint_jacobian_nz[164] = t11 * t40 * t174 * 0.5;
  constraint_jacobian_nz[165] = t4 * t11 * t49 * t176 * -0.5;
  constraint_jacobian_nz[166] = dt * t31 * t171;
  constraint_jacobian_nz[167] = dt * t40 * t174;
  constraint_jacobian_nz[168] = -dt * t4 * t49 * t176;
  constraint_jacobian_nz[169] = -in4[2] * in3[2];
  constraint_jacobian_nz[170] = t11 * t31 * t178 * -0.5;
  constraint_jacobian_nz[171] = t11 * t40 * t181 * -0.5;
  constraint_jacobian_nz[172] = t4 * t11 * t49 * t184 * 0.5;
  constraint_jacobian_nz[173] = -dt * t31 * t178;
  constraint_jacobian_nz[174] = -dt * t40 * t181;
  constraint_jacobian_nz[175] = dt * t4 * t49 * t184;
  constraint_jacobian_nz[176] = -in3[2] * in5[2];
  constraint_jacobian_nz[177] = -in4[2] * in3[2];
  constraint_jacobian_nz[178] = in3[2] * t11 * t100 * -0.5;
  constraint_jacobian_nz[179] = t11 * t31 * t186 * -0.5;
  constraint_jacobian_nz[180] = t11 * t40 * t188 * -0.5;
  constraint_jacobian_nz[181] = t4 * t11 * t26 * t49 * t191 * 0.5;
  constraint_jacobian_nz[182] = -in3[2] * dt * t100;
  constraint_jacobian_nz[183] = -dt * t31 * t186;
  constraint_jacobian_nz[184] = -dt * t40 * t188;
  constraint_jacobian_nz[185] = dt * t4 * t26 * t49 * t191;
  constraint_jacobian_nz[186] = in3[2];
  constraint_jacobian_nz[187] = -in3[2];
  constraint_jacobian_nz[188] = in3[2] * t11 * t100 * -0.5;
  constraint_jacobian_nz[189] = t11 * t31 * t193 * -0.5;
  constraint_jacobian_nz[190] = t11 * t40 * t196 * -0.5;
  constraint_jacobian_nz[191] = t4 * t11 * t49 * t198 * 0.5;
  constraint_jacobian_nz[192] = -in3[2] * dt * t100;
  constraint_jacobian_nz[193] = -dt * t31 * t193;
  constraint_jacobian_nz[194] = -dt * t40 * t196;
  constraint_jacobian_nz[195] = dt * t4 * t49 * t198;
  constraint_jacobian_nz[196] = in3[2];
  constraint_jacobian_nz[197] = -in3[2];
  constraint_jacobian_nz[198] = in3[2] * t11 * t100 * -0.5;
  constraint_jacobian_nz[199] = t11 * t31 * t200 * 0.5;
  constraint_jacobian_nz[200] = t11 * t40 * t203 * 0.5;
  constraint_jacobian_nz[201] = t4 * t11 * t49 * t206 * -0.5;
  constraint_jacobian_nz[202] = -in3[2] * dt * t100;
  constraint_jacobian_nz[203] = dt * t31 * t200;
  constraint_jacobian_nz[204] = dt * t40 * t203;
  constraint_jacobian_nz[205] = -dt * t4 * t49 * t206;
  constraint_jacobian_nz[206] = -in3[2] * in9[2];
  constraint_jacobian_nz[207] = -in3[2] * in9[2];
  constraint_jacobian_nz[208] = -in3[2] * in9[2];
  constraint_jacobian_nz[209] = -in3[2] * in9[2];
  constraint_jacobian_nz[210] = t11 * t31 * (t207 - in2[22] * t5 * t37) * 0.5;
  constraint_jacobian_nz[211] = t11 * t40 * (t208 - in2[22] * t5 * t46) * 0.5;
  constraint_jacobian_nz[212] = t5 * t11 * t26 * t49 * t211 * -0.5;
  constraint_jacobian_nz[213] = dt * t31 * (t207 - in2[22] * t5 * t37);
  constraint_jacobian_nz[214] = dt * t40 * (t208 - in2[22] * t5 * t46);
  constraint_jacobian_nz[215] = -dt * t5 * t26 * t49 * t211;
  constraint_jacobian_nz[216] = -in4[3] * in3[3];
  constraint_jacobian_nz[217] = t11 * t31 * t213 * 0.5;
  constraint_jacobian_nz[218] = t11 * t40 * t216 * 0.5;
  constraint_jacobian_nz[219] = t5 * t11 * t49 * t218 * -0.5;
  constraint_jacobian_nz[220] = dt * t31 * t213;
  constraint_jacobian_nz[221] = dt * t40 * t216;
  constraint_jacobian_nz[222] = -dt * t5 * t49 * t218;
  constraint_jacobian_nz[223] = -in4[3] * in3[3];
  constraint_jacobian_nz[224] = t11 * t31 * t220 * -0.5;
  constraint_jacobian_nz[225] = t11 * t40 * t223 * -0.5;
  constraint_jacobian_nz[226] = t5 * t11 * t49 * t226 * 0.5;
  constraint_jacobian_nz[227] = -dt * t31 * t220;
  constraint_jacobian_nz[228] = -dt * t40 * t223;
  constraint_jacobian_nz[229] = dt * t5 * t49 * t226;
  constraint_jacobian_nz[230] = -in3[3] * in5[3];
  constraint_jacobian_nz[231] = -in4[3] * in3[3];
  constraint_jacobian_nz[232] = in3[3] * t11 * t100 * -0.5;
  constraint_jacobian_nz[233] = t11 * t31 * t228 * -0.5;
  constraint_jacobian_nz[234] = t11 * t40 * t230 * -0.5;
  constraint_jacobian_nz[235] = t5 * t11 * t26 * t49 * t233 * 0.5;
  constraint_jacobian_nz[236] = -in3[3] * dt * t100;
  constraint_jacobian_nz[237] = -dt * t31 * t228;
  constraint_jacobian_nz[238] = -dt * t40 * t230;
  constraint_jacobian_nz[239] = dt * t5 * t26 * t49 * t233;
  constraint_jacobian_nz[240] = in3[3];
  constraint_jacobian_nz[241] = -in3[3];
  constraint_jacobian_nz[242] = in3[3] * t11 * t100 * -0.5;
  constraint_jacobian_nz[243] = t11 * t31 * t235 * -0.5;
  constraint_jacobian_nz[244] = t11 * t40 * t238 * -0.5;
  constraint_jacobian_nz[245] = t5 * t11 * t49 * t240 * 0.5;
  constraint_jacobian_nz[246] = -in3[3] * dt * t100;
  constraint_jacobian_nz[247] = -dt * t31 * t235;
  constraint_jacobian_nz[248] = -dt * t40 * t238;
  constraint_jacobian_nz[249] = dt * t5 * t49 * t240;
  constraint_jacobian_nz[250] = in3[3];
  constraint_jacobian_nz[251] = -in3[3];
  constraint_jacobian_nz[252] = in3[3] * t11 * t100 * -0.5;
  constraint_jacobian_nz[253] = t11 * t31 * t242 * 0.5;
  constraint_jacobian_nz[254] = t11 * t40 * t34 * 0.5;
  constraint_jacobian_nz[255] = t5 * t11 * t49 * t16 * -0.5;
  constraint_jacobian_nz[256] = -in3[3] * dt * t100;
  constraint_jacobian_nz[257] = dt * t31 * t242;
  constraint_jacobian_nz[258] = dt * t40 * t34;
  constraint_jacobian_nz[259] = -dt * t5 * t49 * t16;
  constraint_jacobian_nz[260] = -in3[3] * in9[3];
  constraint_jacobian_nz[261] = -in3[3] * in9[3];
  constraint_jacobian_nz[262] = -in3[3] * in9[3];
  constraint_jacobian_nz[263] = -in3[3] * in9[3];
  constraint_jacobian_nz[264] = 1.0;
  constraint_jacobian_nz[265] = t21;
  constraint_jacobian_nz[266] = t56;
  constraint_jacobian_nz[267] = t8;
  constraint_jacobian_nz[268] = t9;
  constraint_jacobian_nz[269] = 1.0;
  constraint_jacobian_nz[270] = t21;
  constraint_jacobian_nz[271] = t56;
  constraint_jacobian_nz[272] = t8;
  constraint_jacobian_nz[273] = t9;
  constraint_jacobian_nz[274] = 1.0;
  constraint_jacobian_nz[275] = t21;
  constraint_jacobian_nz[276] = t56;
  constraint_jacobian_nz[277] = t8;
  constraint_jacobian_nz[278] = t9;
  constraint_jacobian_nz[279] = 1.0;
  constraint_jacobian_nz[280] = 1.0;
  constraint_jacobian_nz[281] = 1.0;
  constraint_jacobian_nz[282] = 1.0;
  constraint_jacobian_nz[283] = 1.0;
  constraint_jacobian_nz[284] = 1.0;
  constraint_jacobian_nz[285] = 1.0;
  constraint_jacobian_nz[286] = 1.0;
  constraint_jacobian_nz[287] = 1.0;
  constraint_jacobian_nz[288] = t21;
  constraint_jacobian_nz[289] = t21;
  constraint_jacobian_nz[290] = t21;
  constraint_jacobian_nz[291] = t56;
  constraint_jacobian_nz[292] = t56;
  constraint_jacobian_nz[293] = t56;
  constraint_jacobian_nz[294] = t8;
  constraint_jacobian_nz[295] = t8;
  constraint_jacobian_nz[296] = t8;
  constraint_jacobian_nz[297] = t9;
  constraint_jacobian_nz[298] = t9;
  constraint_jacobian_nz[299] = t9;
}

//
// File trailer for RPCConstraintJacobian.cpp
//
// [EOF]
//
