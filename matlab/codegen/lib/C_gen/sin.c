/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sin.c
 *
 * Code generation for function 'sin'
 *
 */

/* Include files */
#include <math.h>
#include "rt_nonfinite.h"
#include "C_gen.h"
#include "G_gen.h"
#include "M_gen.h"
#include "u_model.h"
#include "sin.h"

/* Function Definitions */
void b_sin(emxArray_real_T *x)
{
  int nx;
  int k;
  nx = x->size[1];
  for (k = 0; k < nx; k++) {
    x->data[k] = sin(x->data[k]);
  }
}

/* End of code generation (sin.c) */
