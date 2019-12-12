/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * C_gen.h
 *
 * Code generation for function 'C_gen'
 *
 */

#ifndef C_GEN_H
#define C_GEN_H

/* Include files */
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "C_gen_types.h"

/* Function Declarations */
extern void C_gen(const emxArray_real_T *q, const emxArray_real_T *dq,
                  emxArray_real_T *C);

#endif

/* End of code generation (C_gen.h) */
