/*
 * File: mldivide.c
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 13-Aug-2017 17:54:11
 */

/* Include Files */
#include "QPhild2.h"
#include "get_f.h"
#include "mpc_gain.h"
#include "mldivide.h"

/* Function Definitions */

/*
 * Arguments    : const float A[9]
 *                const float B[36]
 *                float Y[36]
 * Return Type  : void
 */
void b_mldivide(const float A[9], const float B[36], float Y[36])
{
  float b_A[9];
  int rtemp;
  int r1;
  int r2;
  int r3;
  float maxval;
  float a21;
  for (rtemp = 0; rtemp < 9; rtemp++) {
    b_A[rtemp] = A[rtemp];
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabsf(A[0]);
  a21 = fabsf(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabsf(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[3 + r2] -= b_A[r2] * b_A[3 + r1];
  b_A[3 + r3] -= b_A[r3] * b_A[3 + r1];
  b_A[6 + r2] -= b_A[r2] * b_A[6 + r1];
  b_A[6 + r3] -= b_A[r3] * b_A[6 + r1];
  if (fabsf(b_A[3 + r3]) > fabsf(b_A[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[3 + r3] /= b_A[3 + r2];
  b_A[6 + r3] -= b_A[3 + r3] * b_A[6 + r2];
  for (rtemp = 0; rtemp < 12; rtemp++) {
    Y[3 * rtemp] = B[r1 + 3 * rtemp];
    Y[1 + 3 * rtemp] = B[r2 + 3 * rtemp] - Y[3 * rtemp] * b_A[r2];
    Y[2 + 3 * rtemp] = (B[r3 + 3 * rtemp] - Y[3 * rtemp] * b_A[r3]) - Y[1 + 3 *
      rtemp] * b_A[3 + r3];
    Y[2 + 3 * rtemp] /= b_A[6 + r3];
    Y[3 * rtemp] -= Y[2 + 3 * rtemp] * b_A[6 + r1];
    Y[1 + 3 * rtemp] -= Y[2 + 3 * rtemp] * b_A[6 + r2];
    Y[1 + 3 * rtemp] /= b_A[3 + r2];
    Y[3 * rtemp] -= Y[1 + 3 * rtemp] * b_A[3 + r1];
    Y[3 * rtemp] /= b_A[r1];
  }
}

/*
 * Arguments    : const float A[9]
 *                const float B[3]
 *                float Y[3]
 * Return Type  : void
 */
void mldivide(const float A[9], const float B[3], float Y[3])
{
  float b_A[9];
  int r1;
  int r2;
  int r3;
  float maxval;
  float a21;
  int rtemp;
  for (r1 = 0; r1 < 9; r1++) {
    b_A[r1] = A[r1];
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabsf(A[0]);
  a21 = fabsf(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabsf(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[3 + r2] -= b_A[r2] * b_A[3 + r1];
  b_A[3 + r3] -= b_A[r3] * b_A[3 + r1];
  b_A[6 + r2] -= b_A[r2] * b_A[6 + r1];
  b_A[6 + r3] -= b_A[r3] * b_A[6 + r1];
  if (fabsf(b_A[3 + r3]) > fabsf(b_A[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[3 + r3] /= b_A[3 + r2];
  b_A[6 + r3] -= b_A[3 + r3] * b_A[6 + r2];
  Y[1] = B[r2] - B[r1] * b_A[r2];
  Y[2] = (B[r3] - B[r1] * b_A[r3]) - Y[1] * b_A[3 + r3];
  Y[2] /= b_A[6 + r3];
  Y[0] = B[r1] - Y[2] * b_A[6 + r1];
  Y[1] -= Y[2] * b_A[6 + r2];
  Y[1] /= b_A[3 + r2];
  Y[0] -= Y[1] * b_A[3 + r1];
  Y[0] /= b_A[r1];
}

/*
 * File trailer for mldivide.c
 *
 * [EOF]
 */
