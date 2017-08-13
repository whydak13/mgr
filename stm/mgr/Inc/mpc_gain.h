/*
 * File: mpc_gain.h
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 06-Aug-2017 14:21:28
 */

#ifndef __MPC_GAIN_H__
#define __MPC_GAIN_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "get_f_types.h"

/* Function Declarations */
extern void mpc_gain(const float Ap[16], const float Bp[4], const float Cp[8],
                     float Nc, float Np, emxArray_real_T *Phi, emxArray_real_T
                     *Phi_Phi, emxArray_real_T *Phi_F, emxArray_real_T *Phi_R,
                     float A_e[36], float B_e[6], float C_e[12],
                     emxArray_real_T *F);

#endif

/*
 * File trailer for mpc_gain.h
 *
 * [EOF]
 */
