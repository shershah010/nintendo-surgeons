/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * main.c
 *
 * Code generation for function 'main'
 *
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
/* Include files */
#include "rt_nonfinite.h"
#include "C_gen.h"
#include "G_gen.h"
#include "M_gen.h"
#include "u_model.h"
#include "main.h"
#include "C_gen_terminate.h"
#include "C_gen_emxAPI.h"
#include "C_gen_initialize.h"

/* Function Declarations */
static emxArray_real_T *argInit_1xUnbounded_real_T(void);
static emxArray_real_T *argInit_2xUnbounded_real_T(void);
static double argInit_real_T(void);
static void main_C_gen(void);
static void main_G_gen(void);
static void main_M_gen(void);
static void main_u_model(void);

/* Function Definitions */
static emxArray_real_T *argInit_1xUnbounded_real_T(void)
{
  emxArray_real_T *result;
  int idx1;

  /* Set the size of the array.
     Change this size to the value that the application requires. */
  result = emxCreate_real_T(1, 2);

  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result->data[idx1] = argInit_real_T();
  }

  return result;
}

static emxArray_real_T *argInit_2xUnbounded_real_T(void)
{
  emxArray_real_T *result;
  int idx1;

  /* Set the size of the array.
     Change this size to the value that the application requires. */
  result = emxCreate_real_T(2, 2);

  /* Loop over the array to initialize each element. */
  for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result->data[idx1 << 1] = argInit_real_T();
  }

  for (idx1 = 0; idx1 < result->size[1U]; idx1++) {
    /* Set the value of the array element.
       Change this value to the value that the application requires. */
    result->data[1 + (idx1 << 1)] = argInit_real_T();
  }

  return result;
}

static double argInit_real_T(void)
{
  return 0.0;
}

static void main_C_gen(void)
{
  emxArray_real_T *C;
  emxArray_real_T *q;
  emxArray_real_T *dq;
  emxInitArray_real_T(&C, 2);

  /* Initialize function 'C_gen' input arguments. */
  /* Initialize function input argument 'q'. */
  q = argInit_1xUnbounded_real_T();

  /* Initialize function input argument 'dq'. */
  dq = argInit_1xUnbounded_real_T();

  /* Call the entry-point 'C_gen'. */
  C_gen(q, dq, C);
  emxDestroyArray_real_T(C);
  emxDestroyArray_real_T(dq);
  emxDestroyArray_real_T(q);
}

static void main_G_gen(void)
{
  emxArray_real_T *G;
  emxArray_real_T *q;
  emxInitArray_real_T(&G, 2);

  /* Initialize function 'G_gen' input arguments. */
  /* Initialize function input argument 'q'. */
  q = argInit_1xUnbounded_real_T();

  /* Call the entry-point 'G_gen'. */
  G_gen(q, G);
  emxDestroyArray_real_T(G);
  emxDestroyArray_real_T(q);
}

static void main_M_gen(void)
{
  emxArray_real_T *M;
  emxArray_real_T *q;
  emxInitArray_real_T(&M, 2);

  /* Initialize function 'M_gen' input arguments. */
  /* Initialize function input argument 'q'. */
  q = argInit_1xUnbounded_real_T();

  /* Call the entry-point 'M_gen'. */
  M_gen(q, M);
  emxDestroyArray_real_T(M);
  emxDestroyArray_real_T(q);
}

static void main_u_model(void)
{
  emxArray_real_T *u;
  double K_tmp;
  double alpha;
  double b_gamma;
  emxArray_real_T *traj_desired;
  emxInitArray_real_T(&u, 2);

  /* Initialize function 'u_model' input arguments. */
  K_tmp = argInit_real_T();
  alpha = argInit_real_T();
  b_gamma = argInit_real_T();

  /* Initialize function input argument 'traj_desired'. */
  traj_desired = argInit_2xUnbounded_real_T();

  /* Call the entry-point 'u_model'. */
  u_model(K_tmp, K_tmp, alpha, b_gamma, traj_desired, u);
  emxDestroyArray_real_T(u);
  emxDestroyArray_real_T(traj_desired);
}

int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* Initialize the application.
     You do not need to do this more than one time. */
  C_gen_initialize();

  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_C_gen();
  main_G_gen();
  main_M_gen();
  main_u_model();

  /* Terminate the application.
     You do not need to do this more than one time. */
  C_gen_terminate();
  return 0;
}

/* End of code generation (main.c) */
