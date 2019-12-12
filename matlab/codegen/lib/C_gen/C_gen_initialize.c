/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * C_gen_initialize.c
 *
 * Code generation for function 'C_gen_initialize'
 *
 */

/* Include files */
#include "rt_nonfinite.h"
#include "C_gen.h"
#include "G_gen.h"
#include "M_gen.h"
#include "u_model.h"
#include "C_gen_initialize.h"

/* Function Definitions */
void C_gen_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

/* End of code generation (C_gen_initialize.c) */
