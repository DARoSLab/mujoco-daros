/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_matrixFunction_api.h
 *
 * Code generation for function 'matrixFunction'
 *
 */

#ifndef _CODER_MATRIXFUNCTION_API_H
#define _CODER_MATRIXFUNCTION_API_H

/* Include files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void matrixFunction(real_T in1[2], real_T A[4]);
  void matrixFunction_api(const mxArray * const prhs[1], const mxArray *plhs[1]);
  void matrixFunction_atexit(void);
  void matrixFunction_initialize(void);
  void matrixFunction_terminate(void);
  void matrixFunction_xil_shutdown(void);
  void matrixFunction_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/* End of code generation (_coder_matrixFunction_api.h) */
