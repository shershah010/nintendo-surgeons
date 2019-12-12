/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_C_gen_api.h
 *
 * Code generation for function '_coder_C_gen_api'
 *
 */

#ifndef _CODER_C_GEN_API_H
#define _CODER_C_GEN_API_H

/* Include files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_C_gen_api.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void C_gen(emxArray_real_T *q, emxArray_real_T *dq, emxArray_real_T *C);
extern void C_gen_api(const mxArray * const prhs[2], int32_T nlhs, const mxArray
                      *plhs[1]);
extern void C_gen_atexit(void);
extern void C_gen_initialize(void);
extern void C_gen_terminate(void);
extern void C_gen_xil_shutdown(void);
extern void C_gen_xil_terminate(void);
extern void G_gen(emxArray_real_T *q, emxArray_real_T *G);
extern void G_gen_api(const mxArray * const prhs[1], int32_T nlhs, const mxArray
                      *plhs[1]);
extern void M_gen(emxArray_real_T *q, emxArray_real_T *M);
extern void M_gen_api(const mxArray * const prhs[1], int32_T nlhs, const mxArray
                      *plhs[1]);
extern void u_model(real_T K, real_T D, real_T alpha, real_T b_gamma,
                    emxArray_real_T *traj_desired, emxArray_real_T *u);
extern void u_model_api(const mxArray * const prhs[5], int32_T nlhs, const
  mxArray *plhs[1]);

#endif

/* End of code generation (_coder_C_gen_api.h) */
