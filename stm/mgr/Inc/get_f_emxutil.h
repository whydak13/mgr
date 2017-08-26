/*
 * File: get_f_emxutil.h
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 13-Aug-2017 17:54:11
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
extern void b_emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions);
extern void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
extern void emxFree_real32_T(emxArray_real32_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);

#endif

/*
 * File trailer for get_f_emxutil.h
 *
 * [EOF]
 */
