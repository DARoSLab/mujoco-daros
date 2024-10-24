//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RPCConstraintJacobianSP.cpp
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
// RPCCONSTRAINTJACOBIANSP
//     [ROW_INDEX_CJ,COL_INDEX_CJ] = RPCCONSTRAINTJACOBIANSP(ITER,NUM_X,NUM_C)
// Arguments    : int iter
//                int NUM_X
//                int NUM_C
//                int row_index_CJ[300]
//                int col_index_CJ[300]
// Return Type  : void
//
void RPCConstraintJacobianSP(int iter, int NUM_X, int NUM_C, int
  row_index_CJ[300], int col_index_CJ[300])
{
  int t2;

  //     This function was generated by the Symbolic Math Toolbox version 7.1.
  //     05-Mar-2020 07:54:11
  t2 = NUM_C * iter;
  row_index_CJ[0] = t2;
  row_index_CJ[1] = t2 + 16;
  row_index_CJ[2] = t2 + 19;
  row_index_CJ[3] = t2 + 22;
  row_index_CJ[4] = t2 + 25;
  row_index_CJ[5] = t2 + 1;
  row_index_CJ[6] = t2 + 17;
  row_index_CJ[7] = t2 + 20;
  row_index_CJ[8] = t2 + 23;
  row_index_CJ[9] = t2 + 26;
  row_index_CJ[10] = t2 + 2;
  row_index_CJ[11] = t2 + 12;
  row_index_CJ[12] = t2 + 13;
  row_index_CJ[13] = t2 + 14;
  row_index_CJ[14] = t2 + 15;
  row_index_CJ[15] = t2 + 18;
  row_index_CJ[16] = t2 + 21;
  row_index_CJ[17] = t2 + 24;
  row_index_CJ[18] = t2 + 27;
  row_index_CJ[19] = t2 + 3;
  row_index_CJ[20] = t2 + 4;
  row_index_CJ[21] = t2 + 5;
  row_index_CJ[22] = t2 + 9;
  row_index_CJ[23] = t2 + 10;
  row_index_CJ[24] = t2 + 11;
  row_index_CJ[25] = t2 + 3;
  row_index_CJ[26] = t2 + 4;
  row_index_CJ[27] = t2 + 5;
  row_index_CJ[28] = t2 + 9;
  row_index_CJ[29] = t2 + 10;
  row_index_CJ[30] = t2 + 11;
  row_index_CJ[31] = t2 + 3;
  row_index_CJ[32] = t2 + 4;
  row_index_CJ[33] = t2 + 5;
  row_index_CJ[34] = t2 + 9;
  row_index_CJ[35] = t2 + 10;
  row_index_CJ[36] = t2;
  row_index_CJ[37] = t2 + 6;
  row_index_CJ[38] = t2 + 1;
  row_index_CJ[39] = t2 + 7;
  row_index_CJ[40] = t2 + 2;
  row_index_CJ[41] = t2 + 8;
  row_index_CJ[42] = t2 + 3;
  row_index_CJ[43] = t2 + 9;
  row_index_CJ[44] = t2 + 4;
  row_index_CJ[45] = t2 + 10;
  row_index_CJ[46] = t2 + 5;
  row_index_CJ[47] = t2 + 11;
  row_index_CJ[48] = t2 + 3;
  row_index_CJ[49] = t2 + 4;
  row_index_CJ[50] = t2 + 5;
  row_index_CJ[51] = t2 + 9;
  row_index_CJ[52] = t2 + 10;
  row_index_CJ[53] = t2 + 11;
  row_index_CJ[54] = t2 + 16;
  row_index_CJ[55] = t2 + 3;
  row_index_CJ[56] = t2 + 4;
  row_index_CJ[57] = t2 + 5;
  row_index_CJ[58] = t2 + 9;
  row_index_CJ[59] = t2 + 10;
  row_index_CJ[60] = t2 + 11;
  row_index_CJ[61] = t2 + 17;
  row_index_CJ[62] = t2 + 3;
  row_index_CJ[63] = t2 + 4;
  row_index_CJ[64] = t2 + 5;
  row_index_CJ[65] = t2 + 9;
  row_index_CJ[66] = t2 + 10;
  row_index_CJ[67] = t2 + 11;
  row_index_CJ[68] = t2 + 12;
  row_index_CJ[69] = t2 + 18;
  row_index_CJ[70] = t2;
  row_index_CJ[71] = t2 + 3;
  row_index_CJ[72] = t2 + 4;
  row_index_CJ[73] = t2 + 5;
  row_index_CJ[74] = t2 + 6;
  row_index_CJ[75] = t2 + 9;
  row_index_CJ[76] = t2 + 10;
  row_index_CJ[77] = t2 + 11;
  row_index_CJ[78] = t2 + 28;
  row_index_CJ[79] = t2 + 29;
  row_index_CJ[80] = t2 + 1;
  row_index_CJ[81] = t2 + 3;
  row_index_CJ[82] = t2 + 4;
  row_index_CJ[83] = t2 + 5;
  row_index_CJ[84] = t2 + 7;
  row_index_CJ[85] = t2 + 9;
  row_index_CJ[86] = t2 + 10;
  row_index_CJ[87] = t2 + 11;
  row_index_CJ[88] = t2 + 30;
  row_index_CJ[89] = t2 + 31;
  row_index_CJ[90] = t2 + 2;
  row_index_CJ[91] = t2 + 3;
  row_index_CJ[92] = t2 + 4;
  row_index_CJ[93] = t2 + 5;
  row_index_CJ[94] = t2 + 8;
  row_index_CJ[95] = t2 + 9;
  row_index_CJ[96] = t2 + 10;
  row_index_CJ[97] = t2 + 11;
  row_index_CJ[98] = t2 + 28;
  row_index_CJ[99] = t2 + 29;
  row_index_CJ[100] = t2 + 30;
  row_index_CJ[101] = t2 + 31;
  row_index_CJ[102] = t2 + 3;
  row_index_CJ[103] = t2 + 4;
  row_index_CJ[104] = t2 + 5;
  row_index_CJ[105] = t2 + 9;
  row_index_CJ[106] = t2 + 10;
  row_index_CJ[107] = t2 + 11;
  row_index_CJ[108] = t2 + 19;
  row_index_CJ[109] = t2 + 3;
  row_index_CJ[110] = t2 + 4;
  row_index_CJ[111] = t2 + 5;
  row_index_CJ[112] = t2 + 9;
  row_index_CJ[113] = t2 + 10;
  row_index_CJ[114] = t2 + 11;
  row_index_CJ[115] = t2 + 20;
  row_index_CJ[116] = t2 + 3;
  row_index_CJ[117] = t2 + 4;
  row_index_CJ[118] = t2 + 5;
  row_index_CJ[119] = t2 + 9;
  row_index_CJ[120] = t2 + 10;
  row_index_CJ[121] = t2 + 11;
  row_index_CJ[122] = t2 + 13;
  row_index_CJ[123] = t2 + 21;
  row_index_CJ[124] = t2;
  row_index_CJ[125] = t2 + 3;
  row_index_CJ[126] = t2 + 4;
  row_index_CJ[127] = t2 + 5;
  row_index_CJ[128] = t2 + 6;
  row_index_CJ[129] = t2 + 9;
  row_index_CJ[130] = t2 + 10;
  row_index_CJ[131] = t2 + 11;
  row_index_CJ[132] = t2 + 32;
  row_index_CJ[133] = t2 + 33;
  row_index_CJ[134] = t2 + 1;
  row_index_CJ[135] = t2 + 3;
  row_index_CJ[136] = t2 + 4;
  row_index_CJ[137] = t2 + 5;
  row_index_CJ[138] = t2 + 7;
  row_index_CJ[139] = t2 + 9;
  row_index_CJ[140] = t2 + 10;
  row_index_CJ[141] = t2 + 11;
  row_index_CJ[142] = t2 + 34;
  row_index_CJ[143] = t2 + 35;
  row_index_CJ[144] = t2 + 2;
  row_index_CJ[145] = t2 + 3;
  row_index_CJ[146] = t2 + 4;
  row_index_CJ[147] = t2 + 5;
  row_index_CJ[148] = t2 + 8;
  row_index_CJ[149] = t2 + 9;
  row_index_CJ[150] = t2 + 10;
  row_index_CJ[151] = t2 + 11;
  row_index_CJ[152] = t2 + 32;
  row_index_CJ[153] = t2 + 33;
  row_index_CJ[154] = t2 + 34;
  row_index_CJ[155] = t2 + 35;
  row_index_CJ[156] = t2 + 3;
  row_index_CJ[157] = t2 + 4;
  row_index_CJ[158] = t2 + 5;
  row_index_CJ[159] = t2 + 9;
  row_index_CJ[160] = t2 + 10;
  row_index_CJ[161] = t2 + 11;
  row_index_CJ[162] = t2 + 22;
  row_index_CJ[163] = t2 + 3;
  row_index_CJ[164] = t2 + 4;
  row_index_CJ[165] = t2 + 5;
  row_index_CJ[166] = t2 + 9;
  row_index_CJ[167] = t2 + 10;
  row_index_CJ[168] = t2 + 11;
  row_index_CJ[169] = t2 + 23;
  row_index_CJ[170] = t2 + 3;
  row_index_CJ[171] = t2 + 4;
  row_index_CJ[172] = t2 + 5;
  row_index_CJ[173] = t2 + 9;
  row_index_CJ[174] = t2 + 10;
  row_index_CJ[175] = t2 + 11;
  row_index_CJ[176] = t2 + 14;
  row_index_CJ[177] = t2 + 24;
  row_index_CJ[178] = t2;
  row_index_CJ[179] = t2 + 3;
  row_index_CJ[180] = t2 + 4;
  row_index_CJ[181] = t2 + 5;
  row_index_CJ[182] = t2 + 6;
  row_index_CJ[183] = t2 + 9;
  row_index_CJ[184] = t2 + 10;
  row_index_CJ[185] = t2 + 11;
  row_index_CJ[186] = t2 + 36;
  row_index_CJ[187] = t2 + 37;
  row_index_CJ[188] = t2 + 1;
  row_index_CJ[189] = t2 + 3;
  row_index_CJ[190] = t2 + 4;
  row_index_CJ[191] = t2 + 5;
  row_index_CJ[192] = t2 + 7;
  row_index_CJ[193] = t2 + 9;
  row_index_CJ[194] = t2 + 10;
  row_index_CJ[195] = t2 + 11;
  row_index_CJ[196] = t2 + 38;
  row_index_CJ[197] = t2 + 39;
  row_index_CJ[198] = t2 + 2;
  row_index_CJ[199] = t2 + 3;
  row_index_CJ[200] = t2 + 4;
  row_index_CJ[201] = t2 + 5;
  row_index_CJ[202] = t2 + 8;
  row_index_CJ[203] = t2 + 9;
  row_index_CJ[204] = t2 + 10;
  row_index_CJ[205] = t2 + 11;
  row_index_CJ[206] = t2 + 36;
  row_index_CJ[207] = t2 + 37;
  row_index_CJ[208] = t2 + 38;
  row_index_CJ[209] = t2 + 39;
  row_index_CJ[210] = t2 + 3;
  row_index_CJ[211] = t2 + 4;
  row_index_CJ[212] = t2 + 5;
  row_index_CJ[213] = t2 + 9;
  row_index_CJ[214] = t2 + 10;
  row_index_CJ[215] = t2 + 11;
  row_index_CJ[216] = t2 + 25;
  row_index_CJ[217] = t2 + 3;
  row_index_CJ[218] = t2 + 4;
  row_index_CJ[219] = t2 + 5;
  row_index_CJ[220] = t2 + 9;
  row_index_CJ[221] = t2 + 10;
  row_index_CJ[222] = t2 + 11;
  row_index_CJ[223] = t2 + 26;
  row_index_CJ[224] = t2 + 3;
  row_index_CJ[225] = t2 + 4;
  row_index_CJ[226] = t2 + 5;
  row_index_CJ[227] = t2 + 9;
  row_index_CJ[228] = t2 + 10;
  row_index_CJ[229] = t2 + 11;
  row_index_CJ[230] = t2 + 15;
  row_index_CJ[231] = t2 + 27;
  row_index_CJ[232] = t2;
  row_index_CJ[233] = t2 + 3;
  row_index_CJ[234] = t2 + 4;
  row_index_CJ[235] = t2 + 5;
  row_index_CJ[236] = t2 + 6;
  row_index_CJ[237] = t2 + 9;
  row_index_CJ[238] = t2 + 10;
  row_index_CJ[239] = t2 + 11;
  row_index_CJ[240] = t2 + 40;
  row_index_CJ[241] = t2 + 41;
  row_index_CJ[242] = t2 + 1;
  row_index_CJ[243] = t2 + 3;
  row_index_CJ[244] = t2 + 4;
  row_index_CJ[245] = t2 + 5;
  row_index_CJ[246] = t2 + 7;
  row_index_CJ[247] = t2 + 9;
  row_index_CJ[248] = t2 + 10;
  row_index_CJ[249] = t2 + 11;
  row_index_CJ[250] = t2 + 42;
  row_index_CJ[251] = t2 + 43;
  row_index_CJ[252] = t2 + 2;
  row_index_CJ[253] = t2 + 3;
  row_index_CJ[254] = t2 + 4;
  row_index_CJ[255] = t2 + 5;
  row_index_CJ[256] = t2 + 8;
  row_index_CJ[257] = t2 + 9;
  row_index_CJ[258] = t2 + 10;
  row_index_CJ[259] = t2 + 11;
  row_index_CJ[260] = t2 + 40;
  row_index_CJ[261] = t2 + 41;
  row_index_CJ[262] = t2 + 42;
  row_index_CJ[263] = t2 + 43;
  row_index_CJ[264] = t2;
  row_index_CJ[265] = t2 + 16;
  row_index_CJ[266] = t2 + 19;
  row_index_CJ[267] = t2 + 22;
  row_index_CJ[268] = t2 + 25;
  row_index_CJ[269] = t2 + 1;
  row_index_CJ[270] = t2 + 17;
  row_index_CJ[271] = t2 + 20;
  row_index_CJ[272] = t2 + 23;
  row_index_CJ[273] = t2 + 26;
  row_index_CJ[274] = t2 + 2;
  row_index_CJ[275] = t2 + 18;
  row_index_CJ[276] = t2 + 21;
  row_index_CJ[277] = t2 + 24;
  row_index_CJ[278] = t2 + 27;
  row_index_CJ[279] = t2 + 3;
  row_index_CJ[280] = t2 + 4;
  row_index_CJ[281] = t2 + 5;
  row_index_CJ[282] = t2 + 6;
  row_index_CJ[283] = t2 + 7;
  row_index_CJ[284] = t2 + 8;
  row_index_CJ[285] = t2 + 9;
  row_index_CJ[286] = t2 + 10;
  row_index_CJ[287] = t2 + 11;
  row_index_CJ[288] = t2 + 16;
  row_index_CJ[289] = t2 + 17;
  row_index_CJ[290] = t2 + 18;
  row_index_CJ[291] = t2 + 19;
  row_index_CJ[292] = t2 + 20;
  row_index_CJ[293] = t2 + 21;
  row_index_CJ[294] = t2 + 22;
  row_index_CJ[295] = t2 + 23;
  row_index_CJ[296] = t2 + 24;
  row_index_CJ[297] = t2 + 25;
  row_index_CJ[298] = t2 + 26;
  row_index_CJ[299] = t2 + 27;
  t2 = NUM_X * iter;
  col_index_CJ[0] = t2;
  col_index_CJ[1] = t2;
  col_index_CJ[2] = t2;
  col_index_CJ[3] = t2;
  col_index_CJ[4] = t2;
  col_index_CJ[5] = t2 + 1;
  col_index_CJ[6] = t2 + 1;
  col_index_CJ[7] = t2 + 1;
  col_index_CJ[8] = t2 + 1;
  col_index_CJ[9] = t2 + 1;
  col_index_CJ[10] = t2 + 2;
  col_index_CJ[11] = t2 + 2;
  col_index_CJ[12] = t2 + 2;
  col_index_CJ[13] = t2 + 2;
  col_index_CJ[14] = t2 + 2;
  col_index_CJ[15] = t2 + 2;
  col_index_CJ[16] = t2 + 2;
  col_index_CJ[17] = t2 + 2;
  col_index_CJ[18] = t2 + 2;
  col_index_CJ[19] = t2 + 3;
  col_index_CJ[20] = t2 + 3;
  col_index_CJ[21] = t2 + 3;
  col_index_CJ[22] = t2 + 3;
  col_index_CJ[23] = t2 + 3;
  col_index_CJ[24] = t2 + 3;
  col_index_CJ[25] = t2 + 4;
  col_index_CJ[26] = t2 + 4;
  col_index_CJ[27] = t2 + 4;
  col_index_CJ[28] = t2 + 4;
  col_index_CJ[29] = t2 + 4;
  col_index_CJ[30] = t2 + 4;
  col_index_CJ[31] = t2 + 5;
  col_index_CJ[32] = t2 + 5;
  col_index_CJ[33] = t2 + 5;
  col_index_CJ[34] = t2 + 5;
  col_index_CJ[35] = t2 + 5;
  col_index_CJ[36] = t2 + 6;
  col_index_CJ[37] = t2 + 6;
  col_index_CJ[38] = t2 + 7;
  col_index_CJ[39] = t2 + 7;
  col_index_CJ[40] = t2 + 8;
  col_index_CJ[41] = t2 + 8;
  col_index_CJ[42] = t2 + 9;
  col_index_CJ[43] = t2 + 9;
  col_index_CJ[44] = t2 + 10;
  col_index_CJ[45] = t2 + 10;
  col_index_CJ[46] = t2 + 11;
  col_index_CJ[47] = t2 + 11;
  col_index_CJ[48] = t2 + 12;
  col_index_CJ[49] = t2 + 12;
  col_index_CJ[50] = t2 + 12;
  col_index_CJ[51] = t2 + 12;
  col_index_CJ[52] = t2 + 12;
  col_index_CJ[53] = t2 + 12;
  col_index_CJ[54] = t2 + 12;
  col_index_CJ[55] = t2 + 13;
  col_index_CJ[56] = t2 + 13;
  col_index_CJ[57] = t2 + 13;
  col_index_CJ[58] = t2 + 13;
  col_index_CJ[59] = t2 + 13;
  col_index_CJ[60] = t2 + 13;
  col_index_CJ[61] = t2 + 13;
  col_index_CJ[62] = t2 + 14;
  col_index_CJ[63] = t2 + 14;
  col_index_CJ[64] = t2 + 14;
  col_index_CJ[65] = t2 + 14;
  col_index_CJ[66] = t2 + 14;
  col_index_CJ[67] = t2 + 14;
  col_index_CJ[68] = t2 + 14;
  col_index_CJ[69] = t2 + 14;
  col_index_CJ[70] = t2 + 15;
  col_index_CJ[71] = t2 + 15;
  col_index_CJ[72] = t2 + 15;
  col_index_CJ[73] = t2 + 15;
  col_index_CJ[74] = t2 + 15;
  col_index_CJ[75] = t2 + 15;
  col_index_CJ[76] = t2 + 15;
  col_index_CJ[77] = t2 + 15;
  col_index_CJ[78] = t2 + 15;
  col_index_CJ[79] = t2 + 15;
  col_index_CJ[80] = t2 + 16;
  col_index_CJ[81] = t2 + 16;
  col_index_CJ[82] = t2 + 16;
  col_index_CJ[83] = t2 + 16;
  col_index_CJ[84] = t2 + 16;
  col_index_CJ[85] = t2 + 16;
  col_index_CJ[86] = t2 + 16;
  col_index_CJ[87] = t2 + 16;
  col_index_CJ[88] = t2 + 16;
  col_index_CJ[89] = t2 + 16;
  col_index_CJ[90] = t2 + 17;
  col_index_CJ[91] = t2 + 17;
  col_index_CJ[92] = t2 + 17;
  col_index_CJ[93] = t2 + 17;
  col_index_CJ[94] = t2 + 17;
  col_index_CJ[95] = t2 + 17;
  col_index_CJ[96] = t2 + 17;
  col_index_CJ[97] = t2 + 17;
  col_index_CJ[98] = t2 + 17;
  col_index_CJ[99] = t2 + 17;
  col_index_CJ[100] = t2 + 17;
  col_index_CJ[101] = t2 + 17;
  col_index_CJ[102] = t2 + 18;
  col_index_CJ[103] = t2 + 18;
  col_index_CJ[104] = t2 + 18;
  col_index_CJ[105] = t2 + 18;
  col_index_CJ[106] = t2 + 18;
  col_index_CJ[107] = t2 + 18;
  col_index_CJ[108] = t2 + 18;
  col_index_CJ[109] = t2 + 19;
  col_index_CJ[110] = t2 + 19;
  col_index_CJ[111] = t2 + 19;
  col_index_CJ[112] = t2 + 19;
  col_index_CJ[113] = t2 + 19;
  col_index_CJ[114] = t2 + 19;
  col_index_CJ[115] = t2 + 19;
  col_index_CJ[116] = t2 + 20;
  col_index_CJ[117] = t2 + 20;
  col_index_CJ[118] = t2 + 20;
  col_index_CJ[119] = t2 + 20;
  col_index_CJ[120] = t2 + 20;
  col_index_CJ[121] = t2 + 20;
  col_index_CJ[122] = t2 + 20;
  col_index_CJ[123] = t2 + 20;
  col_index_CJ[124] = t2 + 21;
  col_index_CJ[125] = t2 + 21;
  col_index_CJ[126] = t2 + 21;
  col_index_CJ[127] = t2 + 21;
  col_index_CJ[128] = t2 + 21;
  col_index_CJ[129] = t2 + 21;
  col_index_CJ[130] = t2 + 21;
  col_index_CJ[131] = t2 + 21;
  col_index_CJ[132] = t2 + 21;
  col_index_CJ[133] = t2 + 21;
  col_index_CJ[134] = t2 + 22;
  col_index_CJ[135] = t2 + 22;
  col_index_CJ[136] = t2 + 22;
  col_index_CJ[137] = t2 + 22;
  col_index_CJ[138] = t2 + 22;
  col_index_CJ[139] = t2 + 22;
  col_index_CJ[140] = t2 + 22;
  col_index_CJ[141] = t2 + 22;
  col_index_CJ[142] = t2 + 22;
  col_index_CJ[143] = t2 + 22;
  col_index_CJ[144] = t2 + 23;
  col_index_CJ[145] = t2 + 23;
  col_index_CJ[146] = t2 + 23;
  col_index_CJ[147] = t2 + 23;
  col_index_CJ[148] = t2 + 23;
  col_index_CJ[149] = t2 + 23;
  col_index_CJ[150] = t2 + 23;
  col_index_CJ[151] = t2 + 23;
  col_index_CJ[152] = t2 + 23;
  col_index_CJ[153] = t2 + 23;
  col_index_CJ[154] = t2 + 23;
  col_index_CJ[155] = t2 + 23;
  col_index_CJ[156] = t2 + 24;
  col_index_CJ[157] = t2 + 24;
  col_index_CJ[158] = t2 + 24;
  col_index_CJ[159] = t2 + 24;
  col_index_CJ[160] = t2 + 24;
  col_index_CJ[161] = t2 + 24;
  col_index_CJ[162] = t2 + 24;
  col_index_CJ[163] = t2 + 25;
  col_index_CJ[164] = t2 + 25;
  col_index_CJ[165] = t2 + 25;
  col_index_CJ[166] = t2 + 25;
  col_index_CJ[167] = t2 + 25;
  col_index_CJ[168] = t2 + 25;
  col_index_CJ[169] = t2 + 25;
  col_index_CJ[170] = t2 + 26;
  col_index_CJ[171] = t2 + 26;
  col_index_CJ[172] = t2 + 26;
  col_index_CJ[173] = t2 + 26;
  col_index_CJ[174] = t2 + 26;
  col_index_CJ[175] = t2 + 26;
  col_index_CJ[176] = t2 + 26;
  col_index_CJ[177] = t2 + 26;
  col_index_CJ[178] = t2 + 27;
  col_index_CJ[179] = t2 + 27;
  col_index_CJ[180] = t2 + 27;
  col_index_CJ[181] = t2 + 27;
  col_index_CJ[182] = t2 + 27;
  col_index_CJ[183] = t2 + 27;
  col_index_CJ[184] = t2 + 27;
  col_index_CJ[185] = t2 + 27;
  col_index_CJ[186] = t2 + 27;
  col_index_CJ[187] = t2 + 27;
  col_index_CJ[188] = t2 + 28;
  col_index_CJ[189] = t2 + 28;
  col_index_CJ[190] = t2 + 28;
  col_index_CJ[191] = t2 + 28;
  col_index_CJ[192] = t2 + 28;
  col_index_CJ[193] = t2 + 28;
  col_index_CJ[194] = t2 + 28;
  col_index_CJ[195] = t2 + 28;
  col_index_CJ[196] = t2 + 28;
  col_index_CJ[197] = t2 + 28;
  col_index_CJ[198] = t2 + 29;
  col_index_CJ[199] = t2 + 29;
  col_index_CJ[200] = t2 + 29;
  col_index_CJ[201] = t2 + 29;
  col_index_CJ[202] = t2 + 29;
  col_index_CJ[203] = t2 + 29;
  col_index_CJ[204] = t2 + 29;
  col_index_CJ[205] = t2 + 29;
  col_index_CJ[206] = t2 + 29;
  col_index_CJ[207] = t2 + 29;
  col_index_CJ[208] = t2 + 29;
  col_index_CJ[209] = t2 + 29;
  col_index_CJ[210] = t2 + 30;
  col_index_CJ[211] = t2 + 30;
  col_index_CJ[212] = t2 + 30;
  col_index_CJ[213] = t2 + 30;
  col_index_CJ[214] = t2 + 30;
  col_index_CJ[215] = t2 + 30;
  col_index_CJ[216] = t2 + 30;
  col_index_CJ[217] = t2 + 31;
  col_index_CJ[218] = t2 + 31;
  col_index_CJ[219] = t2 + 31;
  col_index_CJ[220] = t2 + 31;
  col_index_CJ[221] = t2 + 31;
  col_index_CJ[222] = t2 + 31;
  col_index_CJ[223] = t2 + 31;
  col_index_CJ[224] = t2 + 32;
  col_index_CJ[225] = t2 + 32;
  col_index_CJ[226] = t2 + 32;
  col_index_CJ[227] = t2 + 32;
  col_index_CJ[228] = t2 + 32;
  col_index_CJ[229] = t2 + 32;
  col_index_CJ[230] = t2 + 32;
  col_index_CJ[231] = t2 + 32;
  col_index_CJ[232] = t2 + 33;
  col_index_CJ[233] = t2 + 33;
  col_index_CJ[234] = t2 + 33;
  col_index_CJ[235] = t2 + 33;
  col_index_CJ[236] = t2 + 33;
  col_index_CJ[237] = t2 + 33;
  col_index_CJ[238] = t2 + 33;
  col_index_CJ[239] = t2 + 33;
  col_index_CJ[240] = t2 + 33;
  col_index_CJ[241] = t2 + 33;
  col_index_CJ[242] = t2 + 34;
  col_index_CJ[243] = t2 + 34;
  col_index_CJ[244] = t2 + 34;
  col_index_CJ[245] = t2 + 34;
  col_index_CJ[246] = t2 + 34;
  col_index_CJ[247] = t2 + 34;
  col_index_CJ[248] = t2 + 34;
  col_index_CJ[249] = t2 + 34;
  col_index_CJ[250] = t2 + 34;
  col_index_CJ[251] = t2 + 34;
  col_index_CJ[252] = t2 + 35;
  col_index_CJ[253] = t2 + 35;
  col_index_CJ[254] = t2 + 35;
  col_index_CJ[255] = t2 + 35;
  col_index_CJ[256] = t2 + 35;
  col_index_CJ[257] = t2 + 35;
  col_index_CJ[258] = t2 + 35;
  col_index_CJ[259] = t2 + 35;
  col_index_CJ[260] = t2 + 35;
  col_index_CJ[261] = t2 + 35;
  col_index_CJ[262] = t2 + 35;
  col_index_CJ[263] = t2 + 35;
  col_index_CJ[264] = t2 + 36;
  col_index_CJ[265] = t2 + 36;
  col_index_CJ[266] = t2 + 36;
  col_index_CJ[267] = t2 + 36;
  col_index_CJ[268] = t2 + 36;
  col_index_CJ[269] = t2 + 37;
  col_index_CJ[270] = t2 + 37;
  col_index_CJ[271] = t2 + 37;
  col_index_CJ[272] = t2 + 37;
  col_index_CJ[273] = t2 + 37;
  col_index_CJ[274] = t2 + 38;
  col_index_CJ[275] = t2 + 38;
  col_index_CJ[276] = t2 + 38;
  col_index_CJ[277] = t2 + 38;
  col_index_CJ[278] = t2 + 38;
  col_index_CJ[279] = t2 + 39;
  col_index_CJ[280] = t2 + 40;
  col_index_CJ[281] = t2 + 41;
  col_index_CJ[282] = t2 + 42;
  col_index_CJ[283] = t2 + 43;
  col_index_CJ[284] = t2 + 44;
  col_index_CJ[285] = t2 + 45;
  col_index_CJ[286] = t2 + 46;
  col_index_CJ[287] = t2 + 47;
  col_index_CJ[288] = t2 + 48;
  col_index_CJ[289] = t2 + 49;
  col_index_CJ[290] = t2 + 50;
  col_index_CJ[291] = t2 + 54;
  col_index_CJ[292] = t2 + 55;
  col_index_CJ[293] = t2 + 56;
  col_index_CJ[294] = t2 + 60;
  col_index_CJ[295] = t2 + 61;
  col_index_CJ[296] = t2 + 62;
  col_index_CJ[297] = t2 + 66;
  col_index_CJ[298] = t2 + 67;
  col_index_CJ[299] = t2 + 68;
}

//
// File trailer for RPCConstraintJacobianSP.cpp
//
// [EOF]
//