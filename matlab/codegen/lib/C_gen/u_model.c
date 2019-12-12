/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * u_model.c
 *
 * Code generation for function 'u_model'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "C_gen.h"
#include "G_gen.h"
#include "M_gen.h"
#include "u_model.h"
#include "C_gen_emxutil.h"
#include "rdivide_helper.h"
#include "diff.h"

/* Function Definitions */
void u_model(double K, double D, double alpha, double b_gamma, const
             emxArray_real_T *traj_desired, emxArray_real_T *u)
{
  emxArray_real_T *q_desired;
  int loop_ub;
  int i4;
  emxArray_real_T *b_traj_desired;
  emxArray_real_T *t_diff;
  emxArray_real_T *q_desired_dot;
  emxArray_real_T *r4;
  emxArray_real_T *r5;
  emxArray_real_T *tau_dot;
  emxArray_real_T *r6;
  emxArray_real_T *r7;
  emxArray_real_T *tau_desired;
  double a;
  double b_a;
  emxInit_real_T(&q_desired, 2);

  /* Assume traj_desired is a 2d trajectory array [t_desired, q_desired]. */
  /* Return feed forward PWM based on generated dynamics model. */
  loop_ub = traj_desired->size[1];
  i4 = q_desired->size[0] * q_desired->size[1];
  q_desired->size[0] = 1;
  q_desired->size[1] = loop_ub;
  emxEnsureCapacity_real_T(q_desired, i4);
  for (i4 = 0; i4 < loop_ub; i4++) {
    q_desired->data[i4] = traj_desired->data[1 + (i4 << 1)] * 3.1415926535897931
      / 90.0;
  }

  emxInit_real_T(&b_traj_desired, 2);
  loop_ub = traj_desired->size[1];
  i4 = b_traj_desired->size[0] * b_traj_desired->size[1];
  b_traj_desired->size[0] = 1;
  b_traj_desired->size[1] = loop_ub;
  emxEnsureCapacity_real_T(b_traj_desired, i4);
  for (i4 = 0; i4 < loop_ub; i4++) {
    b_traj_desired->data[i4] = traj_desired->data[i4 << 1];
  }

  emxInit_real_T(&t_diff, 2);
  emxInit_real_T(&q_desired_dot, 2);
  emxInit_real_T(&r4, 2);
  emxInit_real_T(&r5, 2);
  diff(b_traj_desired, t_diff);
  diff(q_desired, r4);
  b_rdivide_helper(r4, t_diff, r5);
  i4 = q_desired_dot->size[0] * q_desired_dot->size[1];
  q_desired_dot->size[0] = 1;
  q_desired_dot->size[1] = 1 + r5->size[1];
  emxEnsureCapacity_real_T(q_desired_dot, i4);
  q_desired_dot->data[0] = 0.0;
  loop_ub = r5->size[1];
  for (i4 = 0; i4 < loop_ub; i4++) {
    q_desired_dot->data[i4 + 1] = r5->data[i4];
  }

  emxInit_real_T(&tau_dot, 2);
  emxInit_real_T(&r6, 2);
  emxInit_real_T(&r7, 2);
  M_gen(q_desired, r4);
  C_gen(q_desired, q_desired_dot, r5);
  G_gen(q_desired, r6);
  diff(q_desired_dot, tau_dot);
  b_rdivide_helper(tau_dot, t_diff, r7);
  i4 = b_traj_desired->size[0] * b_traj_desired->size[1];
  b_traj_desired->size[0] = 1;
  b_traj_desired->size[1] = 1 + r7->size[1];
  emxEnsureCapacity_real_T(b_traj_desired, i4);
  b_traj_desired->data[0] = 0.0;
  loop_ub = r7->size[1];
  for (i4 = 0; i4 < loop_ub; i4++) {
    b_traj_desired->data[i4 + 1] = r7->data[i4];
  }

  emxFree_real_T(&r7);
  emxInit_real_T(&tau_desired, 2);
  i4 = tau_desired->size[0] * tau_desired->size[1];
  tau_desired->size[0] = 1;
  tau_desired->size[1] = r4->size[1];
  emxEnsureCapacity_real_T(tau_desired, i4);
  loop_ub = r4->size[0] * r4->size[1];
  for (i4 = 0; i4 < loop_ub; i4++) {
    tau_desired->data[i4] = ((r4->data[i4] * b_traj_desired->data[i4] +
      (r5->data[i4] + D) * q_desired_dot->data[i4]) + r6->data[i4]) + K *
      q_desired->data[i4];
  }

  emxFree_real_T(&r6);
  emxFree_real_T(&q_desired_dot);
  emxFree_real_T(&q_desired);
  diff(tau_desired, r4);
  b_rdivide_helper(r4, t_diff, r5);
  i4 = tau_dot->size[0] * tau_dot->size[1];
  tau_dot->size[0] = 1;
  tau_dot->size[1] = 1 + r5->size[1];
  emxEnsureCapacity_real_T(tau_dot, i4);
  tau_dot->data[0] = 0.0;
  loop_ub = r5->size[1];
  for (i4 = 0; i4 < loop_ub; i4++) {
    tau_dot->data[i4 + 1] = r5->data[i4];
  }

  a = b_gamma * b_gamma;
  b_a = 2.0 * b_gamma;
  diff(tau_dot, r4);
  b_rdivide_helper(r4, t_diff, r5);
  i4 = b_traj_desired->size[0] * b_traj_desired->size[1];
  b_traj_desired->size[0] = 1;
  b_traj_desired->size[1] = 1 + r5->size[1];
  emxEnsureCapacity_real_T(b_traj_desired, i4);
  b_traj_desired->data[0] = a * 0.0;
  loop_ub = r5->size[1];
  emxFree_real_T(&r4);
  emxFree_real_T(&t_diff);
  for (i4 = 0; i4 < loop_ub; i4++) {
    b_traj_desired->data[i4 + 1] = a * r5->data[i4];
  }

  emxFree_real_T(&r5);
  i4 = u->size[0] * u->size[1];
  u->size[0] = 1;
  u->size[1] = b_traj_desired->size[1];
  emxEnsureCapacity_real_T(u, i4);
  loop_ub = b_traj_desired->size[0] * b_traj_desired->size[1];
  for (i4 = 0; i4 < loop_ub; i4++) {
    u->data[i4] = ((b_traj_desired->data[i4] + b_a * tau_dot->data[i4]) +
                   tau_desired->data[i4]) / alpha;
  }

  emxFree_real_T(&b_traj_desired);
  emxFree_real_T(&tau_dot);
  emxFree_real_T(&tau_desired);
}

/* End of code generation (u_model.c) */
