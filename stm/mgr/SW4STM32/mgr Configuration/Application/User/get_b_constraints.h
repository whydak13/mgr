/*
 * File: get_b_constraints.h
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 10-Sep-2017 19:02:46
 */

#ifndef __GET_B_CONSTRAINTS_H__
#define __GET_B_CONSTRAINTS_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "get_b_constraints_types.h"

/* Function Declarations */
extern void QPhild2(const float H[9], const float f[3], const float A_cons[36],
                    const float b[12], float eta[3]);
extern emxArray_real_T *emxCreateND_real_T(int numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapperND_real_T(double *data, int
  numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows, int cols);
extern emxArray_real_T *emxCreate_real_T(int rows, int cols);
extern void emxDestroyArray_real_T(emxArray_real_T *emxArray);
extern void emxInitArray_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void get_b_constraints(float max_u_delta, float max_u, float current_u,
  float B_cons[12]);
extern void get_b_constraints_initialize(void);
extern void get_f(float Np, const float r[2], const float F[180], const float
                  Xf[6], const float Phi[90], float f[3]);
extern void mpc_gain(const float Ap[16], const float Bp[4], const float Cp[8],
                     float Nc, float Np, emxArray_real_T *Phi, emxArray_real_T
                     *Phi_Phi, emxArray_real_T *Phi_F, emxArray_real_T *Phi_R,
                     double A_e[36], double B_e[6], double C_e[12],
                     emxArray_real_T *F);

#endif

/*
 * File trailer for get_b_constraints.h
 *
 * [EOF]
 */
