/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * diff.c
 *
 * Code generation for function 'diff'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "C_gen.h"
#include "G_gen.h"
#include "M_gen.h"
#include "u_model.h"
#include "diff.h"
#include "C_gen_emxutil.h"

/* Function Definitions */
void diff(const emxArray_real_T *x, emxArray_real_T *y)
{
  int dimSize;
  int orderForDim;
  int iyLead;
  double work_data_idx_0;
  int m;
  double tmp1;
  double tmp2;
  dimSize = x->size[1];
  if (x->size[1] == 0) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else {
    orderForDim = x->size[1] - 1;
    if (orderForDim >= 1) {
      orderForDim = 1;
    }

    if (orderForDim < 1) {
      y->size[0] = 1;
      y->size[1] = 0;
    } else {
      orderForDim = x->size[1] - 1;
      iyLead = y->size[0] * y->size[1];
      y->size[0] = 1;
      y->size[1] = orderForDim;
      emxEnsureCapacity_real_T(y, iyLead);
      if (y->size[1] != 0) {
        orderForDim = 1;
        iyLead = 0;
        work_data_idx_0 = x->data[0];
        for (m = 2; m <= dimSize; m++) {
          tmp1 = x->data[orderForDim];
          tmp2 = work_data_idx_0;
          work_data_idx_0 = tmp1;
          tmp1 -= tmp2;
          orderForDim++;
          y->data[iyLead] = tmp1;
          iyLead++;
        }
      }
    }
  }
}

/* End of code generation (diff.c) */
