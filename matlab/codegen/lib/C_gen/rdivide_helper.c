/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * rdivide_helper.c
 *
 * Code generation for function 'rdivide_helper'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "C_gen.h"
#include "G_gen.h"
#include "M_gen.h"
#include "u_model.h"
#include "rdivide_helper.h"
#include "C_gen_emxutil.h"

/* Function Definitions */
void b_rdivide_helper(const emxArray_real_T *x, const emxArray_real_T *y,
                      emxArray_real_T *z)
{
  int i2;
  int loop_ub;
  i2 = z->size[0] * z->size[1];
  z->size[0] = 1;
  z->size[1] = x->size[1];
  emxEnsureCapacity_real_T(z, i2);
  loop_ub = x->size[0] * x->size[1];
  for (i2 = 0; i2 < loop_ub; i2++) {
    z->data[i2] = x->data[i2] / y->data[i2];
  }
}

void rdivide_helper(const emxArray_real_T *y, emxArray_real_T *z)
{
  int i0;
  int loop_ub;
  i0 = z->size[0] * z->size[1];
  z->size[0] = 1;
  z->size[1] = y->size[1];
  emxEnsureCapacity_real_T(z, i0);
  loop_ub = y->size[0] * y->size[1];
  for (i0 = 0; i0 < loop_ub; i0++) {
    z->data[i0] = 1.0 / y->data[i0];
  }
}

/* End of code generation (rdivide_helper.c) */
