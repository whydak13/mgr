/*
 * File: get_f.c
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 06-Aug-2017 14:21:28
 */

/* Include Files */
#include "QPhild2.h"
#include "get_f.h"
#include "mpc_gain.h"
#include "get_f_emxutil.h"

/* Function Definitions */

/*
 * UNTITLED Summary of this function goes here
 *    Detailed explanation goes here
 * Arguments    : double Np
 *                const double r[2]
 *                const double F[180]
 *                const double Xf[6]
 *                const double Phi[90]
 *                double f[3]
 * Return Type  : void
 */
void get_f(double Np, const double r[2], const double F[180], const double Xf[6],
           const double Phi[90], double f[3])
{
  emxArray_real_T *r2;
  int itilerow;
  int jcol;
  int nx;
  emxArray_real_T *x;
  emxArray_real_T *y_sp;
  unsigned int varargin_1;
  double b_F[30];
  double b_y_sp[30];
  double c_y_sp[3];
  emxInit_real_T(&r2, 2);

  /* 'get_f:4' r2 = repmat(r, Np , 1); */
  itilerow = r2->size[0] * r2->size[1];
  r2->size[0] = (int)Np;
  r2->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)r2, itilerow, (int)sizeof(double));
  if (!((int)Np == 0)) {
    for (jcol = 0; jcol < 2; jcol++) {
      nx = jcol * (int)Np;
      for (itilerow = 1; itilerow <= (int)Np; itilerow++) {
        r2->data[(nx + itilerow) - 1] = r[jcol];
      }
    }
  }

  emxInit_real_T(&x, 2);

  /* 'get_f:5' y_sp = reshape(r2', size(r2, 1) * size(r2, 2), 1); */
  itilerow = x->size[0] * x->size[1];
  x->size[0] = 2;
  x->size[1] = r2->size[0];
  emxEnsureCapacity((emxArray__common *)x, itilerow, (int)sizeof(double));
  jcol = r2->size[0];
  for (itilerow = 0; itilerow < jcol; itilerow++) {
    for (nx = 0; nx < 2; nx++) {
      x->data[nx + x->size[0] * itilerow] = r2->data[itilerow + r2->size[0] * nx];
    }
  }

  b_emxInit_real_T(&y_sp, 1);
  varargin_1 = (unsigned int)r2->size[0] << 1;
  nx = x->size[1] << 1;
  itilerow = y_sp->size[0];
  y_sp->size[0] = (int)varargin_1;
  emxEnsureCapacity((emxArray__common *)y_sp, itilerow, (int)sizeof(double));
  jcol = 0;
  emxFree_real_T(&r2);
  while (jcol + 1 <= nx) {
    y_sp->data[jcol] = x->data[jcol];
    jcol++;
  }

  emxFree_real_T(&x);

  /* 'get_f:6' y2 = F*Xf; */
  /* 'get_f:7' f= (-(y_sp - y2)'*Phi)'; */
  for (itilerow = 0; itilerow < 30; itilerow++) {
    b_F[itilerow] = 0.0;
    for (nx = 0; nx < 6; nx++) {
      b_F[itilerow] += F[itilerow + 30 * nx] * Xf[nx];
    }
  }

  for (itilerow = 0; itilerow < 30; itilerow++) {
    b_y_sp[itilerow] = -(y_sp->data[itilerow] - b_F[itilerow]);
  }

  emxFree_real_T(&y_sp);
  for (itilerow = 0; itilerow < 3; itilerow++) {
    c_y_sp[itilerow] = 0.0;
    for (nx = 0; nx < 30; nx++) {
      c_y_sp[itilerow] += b_y_sp[nx] * Phi[nx + 30 * itilerow];
    }

    f[itilerow] = c_y_sp[itilerow];
  }
}

/*
 * File trailer for get_f.c
 *
 * [EOF]
 */
