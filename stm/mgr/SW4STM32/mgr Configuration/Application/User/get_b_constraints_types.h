/*
 * File: get_b_constraints_types.h
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 10-Sep-2017 19:02:46
 */

#ifndef __GET_B_CONSTRAINTS_TYPES_H__
#define __GET_B_CONSTRAINTS_TYPES_H__

/* Include Files */
#include "rtwtypes.h"

/* Type Definitions */
#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  double *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real_T*/

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 /*typedef_emxArray_real_T*/
#endif

/*
 * File trailer for get_b_constraints_types.h
 *
 * [EOF]
 */
