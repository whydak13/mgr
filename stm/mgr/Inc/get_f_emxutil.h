/*
 * File: get_f_emxutil.h
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 06-Aug-2017 14:21:28
 */

#ifndef __GET_F_EMXUTIL_H__
#define __GET_F_EMXUTIL_H__

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "get_f_types.h"

/* Function Declarations */
extern void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

/*
 * File trailer for get_f_emxutil.h
 *
 * [EOF]
 */
