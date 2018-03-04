/*
 * File: get_b_constraints.c
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 10-Sep-2017 19:02:46
 */

/* Include Files */
#include "get_b_constraints.h"

/* Type Definitions */
#ifndef struct_emxArray__common
#define struct_emxArray__common

struct emxArray__common
{
  void *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray__common*/

#ifndef typedef_emxArray__common
#define typedef_emxArray__common

typedef struct emxArray__common emxArray__common;

#endif                                 /*typedef_emxArray__common*/

#ifndef struct_emxArray_real32_T
#define struct_emxArray_real32_T

struct emxArray_real32_T
{
  float *data;
  int *size;
  int allocatedSize;
  int numDimensions;
  boolean_T canFreeData;
};

#endif                                 /*struct_emxArray_real32_T*/

#ifndef typedef_emxArray_real32_T
#define typedef_emxArray_real32_T

typedef struct emxArray_real32_T emxArray_real32_T;

#endif                                 /*typedef_emxArray_real32_T*/

/* Function Declarations */
static void b_emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions);
static void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
static void b_mldivide(const float A[9], const float B[36], float Y[36]);
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize);
static void emxFree_real32_T(emxArray_real32_T **pEmxArray);
static void emxFree_real_T(emxArray_real_T **pEmxArray);
static void emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions);
static void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
static void mldivide(const float A[9], const float B[3], float Y[3]);

/* Function Definitions */

/*
 * Arguments    : emxArray_real32_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
static void b_emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions)
{
  emxArray_real32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real32_T *)malloc(sizeof(emxArray_real32_T));
  emxArray = *pEmxArray;
  emxArray->data = (float *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
static void b_emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : const float A[9]
 *                const float B[36]
 *                float Y[36]
 * Return Type  : void
 */
static void b_mldivide(const float A[9], const float B[36], float Y[36])
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
 * Arguments    : emxArray__common *emxArray
 *                int oldNumel
 *                int elementSize
 * Return Type  : void
 */
static void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int
  elementSize)
{
  int newNumel;
  int i;
  void *newData;
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      i <<= 1;
    }

    newData = calloc((unsigned int)i, (unsigned int)elementSize);
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, (unsigned int)(elementSize * oldNumel));
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

/*
 * Arguments    : emxArray_real32_T **pEmxArray
 * Return Type  : void
 */
static void emxFree_real32_T(emxArray_real32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real32_T *)NULL) {
    if (((*pEmxArray)->data != (float *)NULL) && (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_real32_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 * Return Type  : void
 */
static void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (double *)NULL) && (*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

/*
 * Arguments    : emxArray_real32_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
static void emxInit_real32_T(emxArray_real32_T **pEmxArray, int numDimensions)
{
  emxArray_real32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real32_T *)malloc(sizeof(emxArray_real32_T));
  emxArray = *pEmxArray;
  emxArray->data = (float *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
static void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/*
 * Arguments    : const float A[9]
 *                const float B[3]
 *                float Y[3]
 * Return Type  : void
 */
static void mldivide(const float A[9], const float B[3], float Y[3])
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
 * E=H;
 *  F=f;
 *  M=A_cons;
 *  gamma=b;
 *  eta =x
 * Arguments    : const float H[9]
 *                const float f[3]
 *                const float A_cons[36]
 *                const float b[12]
 *                float eta[3]
 * Return Type  : void
 */
void QPhild2(const float H[9], const float f[3], const float A_cons[36], const
             float b[12], float eta[3])
{
  float b_H[9];
  int i3;
  double kk;
  int i;
  float f1;
  float b_A_cons[36];
  float b_b[36];
  float P[144];
  int km;
  float c_b[3];
  float d[12];
  double lambda[12];
  boolean_T exitg1;
  double lambda_p[12];
  float la;
  double d0;

  /* 'QPhild2:7' [n1,m1]=size(A_cons); */
  /* 'QPhild2:8' eta=-H\f; */
  for (i3 = 0; i3 < 9; i3++) {
    b_H[i3] = -H[i3];
  }

  mldivide(b_H, f, eta);

  /* 'QPhild2:9' kk=0; */
  kk = 0.0;

  /* 'QPhild2:10' for i=1:n1 */
  for (i = 0; i < 12; i++) {
    /* 'QPhild2:11' if (A_cons(i,:)*eta>b(i)) */
    f1 = 0.0F;
    for (i3 = 0; i3 < 3; i3++) {
      f1 += A_cons[i + 12 * i3] * eta[i3];
    }

    if (f1 > b[i]) {
      /* 'QPhild2:12' kk=kk+1; */
      kk++;
    } else {
      /* 'QPhild2:13' else */
      /* 'QPhild2:14' kk=kk+0; */
    }
  }

  /* 'QPhild2:17' if (kk==0) */
  if (kk == 0.0) {
  } else {
    /* 'QPhild2:21' P=A_cons*(H\A_cons'); */
    for (i3 = 0; i3 < 12; i3++) {
      for (i = 0; i < 3; i++) {
        b_A_cons[i + 3 * i3] = A_cons[i3 + 12 * i];
      }
    }

    b_mldivide(H, b_A_cons, b_b);
    for (i3 = 0; i3 < 12; i3++) {
      for (i = 0; i < 12; i++) {
        P[i3 + 12 * i] = 0.0F;
        for (km = 0; km < 3; km++) {
          P[i3 + 12 * i] += A_cons[i3 + 12 * km] * b_b[km + 3 * i];
        }
      }
    }

    /* 'QPhild2:22' d=(A_cons*(H\f)+b); */
    mldivide(H, f, c_b);
    for (i3 = 0; i3 < 12; i3++) {
      f1 = 0.0F;
      for (i = 0; i < 3; i++) {
        f1 += A_cons[i3 + 12 * i] * c_b[i];
      }

      d[i3] = f1 + b[i3];
    }

    /* 'QPhild2:23' [n,m]=size(d); */
    /* 'QPhild2:24' x_ini=zeros(n,m); */
    /* 'QPhild2:25' lambda=x_ini; */
    for (i = 0; i < 12; i++) {
      lambda[i] = 0.0;
    }

    /* 'QPhild2:26' al=10; */
    /* 'QPhild2:27' for km=1:38 */
    km = 0;
    exitg1 = false;
    while ((!exitg1) && (km < 38)) {//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<,
      /* find the elements in the solution vector one by one */
      /*  km could be larger if the Lagranger multiplier has a slow */
      /*  convergence rate. */
      /* 'QPhild2:31' lambda_p=lambda; */
      memcpy(&lambda_p[0], &lambda[0], 12U * sizeof(double));

      /* 'QPhild2:32' for i=1:n */
      for (i = 0; i < 12; i++) {
        /* 'QPhild2:33' w= P(i,:)*lambda-P(i,i)*lambda(i,1); */
        /* 'QPhild2:34' w=w+d(i,1); */
        /* 'QPhild2:35' la=-w/P(i,i); */
        f1 = 0.0F;
        for (i3 = 0; i3 < 12; i3++) {
          f1 += P[i + 12 * i3] * (float)lambda[i3];
        }

        la = -((f1 - P[i + 12 * i] * (float)lambda[i]) + d[i]) / P[i + 12 * i];

        /* 'QPhild2:36' lambda(i,1)=max(0,la); */
        if (0.0F < la) {
          lambda[i] = la;
        } else {
          lambda[i] = 0.0F;
        }
      }

      /* 'QPhild2:38' al=(lambda-lambda_p)'*(lambda-lambda_p); */
      /* 'QPhild2:39' if (al<10e-8) */
      d0 = 0.0;
      for (i3 = 0; i3 < 12; i3++) {
        d0 += (lambda[i3] - lambda_p[i3]) * (lambda[i3] - lambda_p[i3]);
      }

      if (d0 < 1.0E-7) {
        exitg1 = true;
      } else {
        km++;
      }
    }

    /* 'QPhild2:43' eta=-H\f -H\A_cons'*lambda; */
    for (i3 = 0; i3 < 12; i3++) {
      for (i = 0; i < 3; i++) {
        b_A_cons[i + 3 * i3] = A_cons[i3 + 12 * i];
      }
    }

    b_mldivide(H, b_A_cons, b_b);
    for (i3 = 0; i3 < 9; i3++) {
      b_H[i3] = -H[i3];
    }

    mldivide(b_H, f, c_b);
    for (i3 = 0; i3 < 3; i3++) {
      f1 = 0.0F;
      for (i = 0; i < 12; i++) {
        f1 += b_b[i3 + 3 * i] * (float)lambda[i];
      }

      eta[i3] = c_b[i3] - f1;
    }
  }
}

/*
 * Arguments    : int numDimensions
 *                int *size
 * Return Type  : emxArray_real_T *
 */
emxArray_real_T *emxCreateND_real_T(int numDimensions, int *size)
{
  emxArray_real_T *emx;
  int numEl;
  int i;
  emxInit_real_T(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = (double *)calloc((unsigned int)numEl, sizeof(double));
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  return emx;
}

/*
 * Arguments    : double *data
 *                int numDimensions
 *                int *size
 * Return Type  : emxArray_real_T *
 */
emxArray_real_T *emxCreateWrapperND_real_T(double *data, int numDimensions, int *
  size)
{
  emxArray_real_T *emx;
  int numEl;
  int i;
  emxInit_real_T(&emx, numDimensions);
  numEl = 1;
  for (i = 0; i < numDimensions; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = data;
  emx->numDimensions = numDimensions;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

/*
 * Arguments    : double *data
 *                int rows
 *                int cols
 * Return Type  : emxArray_real_T *
 */
emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows, int cols)
{
  emxArray_real_T *emx;
  int size[2];
  int numEl;
  int i;
  size[0] = rows;
  size[1] = cols;
  emxInit_real_T(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = data;
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  emx->canFreeData = false;
  return emx;
}

/*
 * Arguments    : int rows
 *                int cols
 * Return Type  : emxArray_real_T *
 */
emxArray_real_T *emxCreate_real_T(int rows, int cols)
{
  emxArray_real_T *emx;
  int size[2];
  int numEl;
  int i;
  size[0] = rows;
  size[1] = cols;
  emxInit_real_T(&emx, 2);
  numEl = 1;
  for (i = 0; i < 2; i++) {
    numEl *= size[i];
    emx->size[i] = size[i];
  }

  emx->data = (double *)calloc((unsigned int)numEl, sizeof(double));
  emx->numDimensions = 2;
  emx->allocatedSize = numEl;
  return emx;
}

/*
 * Arguments    : emxArray_real_T *emxArray
 * Return Type  : void
 */
void emxDestroyArray_real_T(emxArray_real_T *emxArray)
{
  emxFree_real_T(&emxArray);
}

/*
 * Arguments    : emxArray_real_T **pEmxArray
 *                int numDimensions
 * Return Type  : void
 */
void emxInitArray_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxInit_real_T(pEmxArray, numDimensions);
}

/*
 * Arguments    : float max_u_delta
 *                float max_u
 *                float current_u
 *                float B_cons[12]
 * Return Type  : void
 */
void get_b_constraints(float max_u_delta, float max_u, float acceleration, float
  b[12])
{
  int i;

  /* 'get_b_constraints:2' Nc=3; */
  /* 'get_b_constraints:3' B_cons = [max_u_delta * ones(Nc*2,1)- current_u */
  /* 'get_b_constraints:4'           max_u * ones(Nc*2,1)+ current_u]; */
  b[0]=777;
	for (i = 0; i < 6; i++) {
			   b[i] = max_u + acceleration;
			 }

	for (i = 0; i < 6; i++) {
			 b[i + 6] = +max_u_delta -acceleration ;
	}

}

/*
 * Arguments    : void
 * Return Type  : void
 */
void get_b_constraints_initialize(void)
{
}

/*
 * UNTITLED Summary of this function goes here
 *    Detailed explanation goes here
 * Arguments    : float Np
 *                const float r[2]
 *                const float F[180]
 *                const float Xf[6]
 *                const float Phi[90]
 *                float f[3]
 * Return Type  : void
 */
void get_f(float Np, const float r[2], const float F[180], const float Xf[6],
           const float Phi[90], float f[3])
{
  emxArray_real32_T *r2;
  int itilerow;
  int jcol;
  int nx;
  emxArray_real32_T *x;
  emxArray_real32_T *y_sp;
  unsigned int varargin_1;
  float b_F[30];
  float b_y_sp[30];
  float c_y_sp[3];
  emxInit_real32_T(&r2, 2);

  /* 'get_f:4' r2 = repmat(r, Np , 1); */
  itilerow = r2->size[0] * r2->size[1];
  r2->size[0] = (int)Np;
  r2->size[1] = 2;
  emxEnsureCapacity((emxArray__common *)r2, itilerow, (int)sizeof(float));
  if (!((int)Np == 0)) {
    for (jcol = 0; jcol < 2; jcol++) {
      nx = jcol * (int)Np;
      for (itilerow = 1; itilerow <= (int)Np; itilerow++) {
        r2->data[(nx + itilerow) - 1] = r[jcol];
      }
    }
  }

  emxInit_real32_T(&x, 2);

  /* 'get_f:5' y_sp = reshape(r2', size(r2, 1) * size(r2, 2), 1); */
  itilerow = x->size[0] * x->size[1];
  x->size[0] = 2;
  x->size[1] = r2->size[0];
  emxEnsureCapacity((emxArray__common *)x, itilerow, (int)sizeof(float));
  jcol = r2->size[0];
  for (itilerow = 0; itilerow < jcol; itilerow++) {
    for (nx = 0; nx < 2; nx++) {
      x->data[nx + x->size[0] * itilerow] = r2->data[itilerow + r2->size[0] * nx];
    }
  }

  b_emxInit_real32_T(&y_sp, 1);
  varargin_1 = (unsigned int)r2->size[0] << 1;
  nx = x->size[1] << 1;
  itilerow = y_sp->size[0];
  y_sp->size[0] = (int)varargin_1;
  emxEnsureCapacity((emxArray__common *)y_sp, itilerow, (int)sizeof(float));
  jcol = 0;
  emxFree_real32_T(&r2);
  while (jcol + 1 <= nx) {
    y_sp->data[jcol] = x->data[jcol];
    jcol++;
  }

  emxFree_real32_T(&x);

  /* 'get_f:6' y2 = F*Xf; */
  /* 'get_f:7' f= (-(y_sp - y2)'*Phi)'; */
  for (itilerow = 0; itilerow < 30; itilerow++) {
    b_F[itilerow] = 0.0F;
    for (nx = 0; nx < 6; nx++) {
      b_F[itilerow] += F[itilerow + 30 * nx] * Xf[nx];
    }
  }

  for (itilerow = 0; itilerow < 30; itilerow++) {
    b_y_sp[itilerow] = -(y_sp->data[itilerow] - b_F[itilerow]);
  }

  emxFree_real32_T(&y_sp);
  for (itilerow = 0; itilerow < 3; itilerow++) {
    c_y_sp[itilerow] = 0.0F;
    for (nx = 0; nx < 30; nx++) {
      c_y_sp[itilerow] += b_y_sp[nx] * Phi[nx + 30 * itilerow];
    }

    f[itilerow] = c_y_sp[itilerow];
  }
}

/*
 * Arguments    : const float Ap[16]
 *                const float Bp[4]
 *                const float Cp[8]
 *                float Nc
 *                float Np
 *                emxArray_real_T *Phi
 *                emxArray_real_T *Phi_Phi
 *                emxArray_real_T *Phi_F
 *                emxArray_real_T *Phi_R
 *                double A_e[36]
 *                double B_e[6]
 *                double C_e[12]
 *                emxArray_real_T *F
 * Return Type  : void
 */
void mpc_gain(const float Ap[16], const float Bp[4], const float Cp[8], float Nc,
              float Np, emxArray_real_T *Phi, emxArray_real_T *Phi_Phi,
              emxArray_real_T *Phi_F, emxArray_real_T *Phi_R, double A_e[36],
              double B_e[6], double C_e[12], emxArray_real_T *F)
{
  static const double dv0[36] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };

  int i0;
  int i1;
  float f0;
  int i2;
  int k;
  static const signed char iv0[4] = { 1, 0, 0, 1 };

  emxArray_real_T *h;
  float y;
  int cr;
  int kk;
  emxArray_real_T *C;
  int m;
  int c;
  int ic;
  int br;
  int ar;
  int ib;
  int ia;
  emxArray_real_T *v;
  unsigned int unnamed_idx_0;
  emxArray_real_T *a;
  unsigned int unnamed_idx_1;

  /* 'mpc_gain:2' [m1,n1]=size(Cp); */
  /* 'mpc_gain:3' [n1,n_in]=size(Bp); */
  /* 'mpc_gain:4' A_e=eye(n1+m1,n1+m1); */
  memcpy(&A_e[0], &dv0[0], 36U * sizeof(double));

  /* 'mpc_gain:5' A_e(1:n1,1:n1)=Ap; */
  for (i0 = 0; i0 < 4; i0++) {
    for (i1 = 0; i1 < 4; i1++) {
      A_e[i1 + 6 * i0] = Ap[i1 + (i0 << 2)];
    }
  }

  /* 'mpc_gain:6' A_e(n1+1:n1+m1,1:n1)=Cp*Ap; */
  for (i0 = 0; i0 < 2; i0++) {
    for (i1 = 0; i1 < 4; i1++) {
      f0 = 0.0F;
      for (i2 = 0; i2 < 4; i2++) {
        f0 += Cp[i0 + (i2 << 1)] * Ap[i2 + (i1 << 2)];
      }

      A_e[(i0 + 6 * i1) + 4] = f0;
    }
  }

  /* 'mpc_gain:7' B_e=zeros(n1+m1,n_in); */
  for (k = 0; k < 6; k++) {
    B_e[k] = 0.0;
  }

  /* 'mpc_gain:8' B_e(1:n1,:)=Bp; */
  for (i0 = 0; i0 < 4; i0++) {
    B_e[i0] = Bp[i0];
  }

  /* 'mpc_gain:9' B_e(n1+1:n1+m1,:)=Cp*Bp; */
  for (i0 = 0; i0 < 2; i0++) {
    f0 = 0.0F;
    for (i1 = 0; i1 < 4; i1++) {
      f0 += Cp[i0 + (i1 << 1)] * Bp[i1];
    }

    B_e[4 + i0] = f0;
  }

  /* 'mpc_gain:10' C_e=zeros(m1,n1+m1); */
  for (i0 = 0; i0 < 12; i0++) {
    C_e[i0] = 0.0;
  }

  /* 'mpc_gain:11' C_e(:,n1+1:n1+m1)=eye(m1,m1); */
  for (i0 = 0; i0 < 2; i0++) {
    for (i1 = 0; i1 < 2; i1++) {
      C_e[i1 + ((4 + i0) << 1)] = iv0[i1 + (i0 << 1)];
    }
  }

  emxInit_real_T(&h, 2);

  /* 'mpc_gain:12' n=n1+m1; */
  /* 'mpc_gain:13' h=zeros(m1*Np,n); */
  y = 2.0F * Np;
  i0 = h->size[0] * h->size[1];
  h->size[0] = (int)y;
  h->size[1] = 6;
  emxEnsureCapacity((emxArray__common *)h, i0, (int)sizeof(double));
  cr = (int)y * 6;
  for (i0 = 0; i0 < cr; i0++) {
    h->data[i0] = 0.0;
  }

  /* 'mpc_gain:14' F=zeros(m1*Np,n); */
  y = 2.0F * Np;
  i0 = F->size[0] * F->size[1];
  F->size[0] = (int)y;
  F->size[1] = 6;
  emxEnsureCapacity((emxArray__common *)F, i0, (int)sizeof(double));
  cr = (int)y * 6;
  for (i0 = 0; i0 < cr; i0++) {
    F->data[i0] = 0.0;
  }

  /* 'mpc_gain:15' h(1:m1,:)=C_e; */
  for (i0 = 0; i0 < 6; i0++) {
    for (i1 = 0; i1 < 2; i1++) {
      h->data[i1 + h->size[0] * i0] = C_e[i1 + (i0 << 1)];
    }
  }

  /* 'mpc_gain:16' F(1:m1,:)=C_e*A_e; */
  for (i0 = 0; i0 < 2; i0++) {
    for (i1 = 0; i1 < 6; i1++) {
      F->data[i0 + F->size[0] * i1] = 0.0;
      for (i2 = 0; i2 < 6; i2++) {
        F->data[i0 + F->size[0] * i1] += C_e[i0 + (i2 << 1)] * A_e[i2 + 6 * i1];
      }
    }
  }

  /* 'mpc_gain:18' for kk=2:Np */
  kk = 0;
  emxInit_real_T(&C, 2);
  while (kk <= (int)(Np + -1.0F) - 1) {
    /* <------------- */
    /* 'mpc_gain:19' h((m1*(kk-1)+1):m1*(kk),:)=h((m1*(kk-2)+1):m1*(kk-1),:)*A_e; */
    y = 2.0F * ((2.0F + (float)kk) - 2.0F);
    f0 = 2.0F * ((2.0F + (float)kk) - 1.0F);
    if (y + 1.0F > f0) {
      i0 = 1;
      i1 = 1;
    } else {
      i0 = (int)(y + 1.0F);
      i1 = (int)f0 + 1;
    }

    y = 2.0F * ((2.0F + (float)kk) - 1.0F);
    if (y + 1.0F > 2.0F * (2.0F + (float)kk)) {
      i2 = 0;
    } else {
      i2 = (int)(y + 1.0F) - 1;
    }

    m = i1 - i0;
    k = C->size[0] * C->size[1];
    C->size[0] = i1 - i0;
    C->size[1] = 6;
    emxEnsureCapacity((emxArray__common *)C, k, (int)sizeof(double));
    cr = (i1 - i0) * 6;
    for (k = 0; k < cr; k++) {
      C->data[k] = 0.0;
    }

    if (i1 - i0 == 0) {
    } else {
      c = (i1 - i0) * 5;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        k = cr + m;
        for (ic = cr; ic + 1 <= k; ic++) {
          C->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          if (A_e[ib] != 0.0) {
            ia = ar;
            k = cr + m;
            for (ic = cr; ic + 1 <= k; ic++) {
              ia++;
              C->data[ic] += A_e[ib] * h->data[((i0 + ia % (i1 - i0)) + h->size
                [0] * (ia / (i1 - i0))) - 1];
            }
          }

          ar += m;
        }

        br += 6;
        cr += m;
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      cr = C->size[0];
      for (i1 = 0; i1 < cr; i1++) {
        h->data[(i2 + i1) + h->size[0] * i0] = C->data[i1 + C->size[0] * i0];
      }
    }

    /* 'mpc_gain:20' F((m1*(kk-1)+1):m1*(kk),:)= F((m1*(kk-2)+1):m1*(kk-1),:)*A_e; */
    y = 2.0F * ((2.0F + (float)kk) - 2.0F);
    f0 = 2.0F * ((2.0F + (float)kk) - 1.0F);
    if (y + 1.0F > f0) {
      i0 = 1;
      i1 = 1;
    } else {
      i0 = (int)(y + 1.0F);
      i1 = (int)f0 + 1;
    }

    y = 2.0F * ((2.0F + (float)kk) - 1.0F);
    if (y + 1.0F > 2.0F * (2.0F + (float)kk)) {
      i2 = 0;
    } else {
      i2 = (int)(y + 1.0F) - 1;
    }

    m = i1 - i0;
    k = C->size[0] * C->size[1];
    C->size[0] = i1 - i0;
    C->size[1] = 6;
    emxEnsureCapacity((emxArray__common *)C, k, (int)sizeof(double));
    cr = (i1 - i0) * 6;
    for (k = 0; k < cr; k++) {
      C->data[k] = 0.0;
    }

    if (i1 - i0 == 0) {
    } else {
      c = (i1 - i0) * 5;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        k = cr + m;
        for (ic = cr; ic + 1 <= k; ic++) {
          C->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        ar = -1;
        for (ib = br; ib + 1 <= br + 6; ib++) {
          if (A_e[ib] != 0.0) {
            ia = ar;
            k = cr + m;
            for (ic = cr; ic + 1 <= k; ic++) {
              ia++;
              C->data[ic] += A_e[ib] * F->data[((i0 + ia % (i1 - i0)) + F->size
                [0] * (ia / (i1 - i0))) - 1];
            }
          }

          ar += m;
        }

        br += 6;
        cr += m;
      }
    }

    for (i0 = 0; i0 < 6; i0++) {
      cr = C->size[0];
      for (i1 = 0; i1 < cr; i1++) {
        F->data[(i2 + i1) + F->size[0] * i0] = C->data[i1 + C->size[0] * i0];
      }
    }

    kk++;
  }

  emxFree_real_T(&C);
  b_emxInit_real_T(&v, 1);

  /*  for kk=2:Np  %<------------- */
  /*      h(kk,:)=h(kk-1,:)*A_e; */
  /*      F(kk,:)= F(kk-1,:)*A_e; */
  /*  end */
  /* 'mpc_gain:27' v=h*B_e; */
  unnamed_idx_0 = (unsigned int)h->size[0];
  m = h->size[0];
  i0 = v->size[0];
  v->size[0] = (int)unnamed_idx_0;
  emxEnsureCapacity((emxArray__common *)v, i0, (int)sizeof(double));
  cr = (int)unnamed_idx_0;
  for (i0 = 0; i0 < cr; i0++) {
    v->data[i0] = 0.0;
  }

  if (h->size[0] == 0) {
  } else {
    cr = 0;
    while ((m > 0) && (cr <= 0)) {
      for (ic = 1; ic <= m; ic++) {
        v->data[ic - 1] = 0.0;
      }

      cr = m;
    }

    br = 0;
    cr = 0;
    while ((m > 0) && (cr <= 0)) {
      ar = -1;
      for (ib = br; ib + 1 <= br + 6; ib++) {
        if (B_e[ib] != 0.0) {
          ia = ar;
          for (ic = 0; ic + 1 <= m; ic++) {
            ia++;
            v->data[ic] += B_e[ib] * h->data[ia];
          }
        }

        ar += m;
      }

      br += 6;
      cr = m;
    }
  }

  emxFree_real_T(&h);

  /* 'mpc_gain:28' Phi=zeros(Np*m1,Nc); */
  y = Np * 2.0F;
  i0 = Phi->size[0] * Phi->size[1];
  Phi->size[0] = (int)y;
  Phi->size[1] = (int)Nc;
  emxEnsureCapacity((emxArray__common *)Phi, i0, (int)sizeof(double));
  cr = (int)y * (int)Nc;
  for (i0 = 0; i0 < cr; i0++) {
    Phi->data[i0] = 0.0;
  }

  /* declare the dimension of Phi */
  /* 'mpc_gain:29' Phi(:,1)=v; */
  cr = v->size[0];
  for (i0 = 0; i0 < cr; i0++) {
    Phi->data[i0] = v->data[i0];
  }

  /*  first column of Phi */
  /* 'mpc_gain:31' for i=2:Nc */
  for (k = 0; k < (int)(Nc + -1.0F); k++) {
    /* 'mpc_gain:32' Phi(:,i)=[zeros(m1*(i-1),1);v(1:m1*(Np-i+1),1)]; */
    f0 = 2.0F * ((Np - (2.0F + (float)k)) + 1.0F);
    if (1.0F > f0) {
      cr = -1;
    } else {
      cr = (int)f0 - 1;
    }

    y = 2.0F * ((2.0F + (float)k) - 1.0F);
    kk = (int)y;
    for (i0 = 0; i0 < kk; i0++) {
      Phi->data[i0 + Phi->size[0] * ((int)(2.0F + (float)k) - 1)] = 0.0;
    }

    for (i0 = 0; i0 <= cr; i0++) {
      Phi->data[(i0 + (int)y) + Phi->size[0] * ((int)(2.0F + (float)k) - 1)] =
        v->data[i0];
    }

    /* Toeplitz matrix */
  }

  emxFree_real_T(&v);
  emxInit_real_T(&a, 2);

  /* 'mpc_gain:35' BarRs=ones(Np*m1,m1); */
  /* 'mpc_gain:36' Phi_Phi= Phi'*Phi; */
  i0 = a->size[0] * a->size[1];
  a->size[0] = Phi->size[1];
  a->size[1] = Phi->size[0];
  emxEnsureCapacity((emxArray__common *)a, i0, (int)sizeof(double));
  cr = Phi->size[0];
  for (i0 = 0; i0 < cr; i0++) {
    kk = Phi->size[1];
    for (i1 = 0; i1 < kk; i1++) {
      a->data[i1 + a->size[0] * i0] = Phi->data[i0 + Phi->size[0] * i1];
    }
  }

  if ((a->size[1] == 1) || (Phi->size[0] == 1)) {
    i0 = Phi_Phi->size[0] * Phi_Phi->size[1];
    Phi_Phi->size[0] = a->size[0];
    Phi_Phi->size[1] = Phi->size[1];
    emxEnsureCapacity((emxArray__common *)Phi_Phi, i0, (int)sizeof(double));
    cr = a->size[0];
    for (i0 = 0; i0 < cr; i0++) {
      kk = Phi->size[1];
      for (i1 = 0; i1 < kk; i1++) {
        Phi_Phi->data[i0 + Phi_Phi->size[0] * i1] = 0.0;
        k = a->size[1];
        for (i2 = 0; i2 < k; i2++) {
          Phi_Phi->data[i0 + Phi_Phi->size[0] * i1] += a->data[i0 + a->size[0] *
            i2] * Phi->data[i2 + Phi->size[0] * i1];
        }
      }
    }
  } else {
    k = a->size[1];
    unnamed_idx_0 = (unsigned int)a->size[0];
    unnamed_idx_1 = (unsigned int)Phi->size[1];
    m = a->size[0];
    i0 = Phi_Phi->size[0] * Phi_Phi->size[1];
    Phi_Phi->size[0] = (int)unnamed_idx_0;
    emxEnsureCapacity((emxArray__common *)Phi_Phi, i0, (int)sizeof(double));
    i0 = Phi_Phi->size[0] * Phi_Phi->size[1];
    Phi_Phi->size[1] = (int)unnamed_idx_1;
    emxEnsureCapacity((emxArray__common *)Phi_Phi, i0, (int)sizeof(double));
    cr = (int)unnamed_idx_0 * (int)unnamed_idx_1;
    for (i0 = 0; i0 < cr; i0++) {
      Phi_Phi->data[i0] = 0.0;
    }

    if ((a->size[0] == 0) || (Phi->size[1] == 0)) {
    } else {
      c = a->size[0] * (Phi->size[1] - 1);
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        i0 = cr + m;
        for (ic = cr; ic + 1 <= i0; ic++) {
          Phi_Phi->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        ar = -1;
        i0 = br + k;
        for (ib = br; ib + 1 <= i0; ib++) {
          if (Phi->data[ib] != 0.0) {
            ia = ar;
            i1 = cr + m;
            for (ic = cr; ic + 1 <= i1; ic++) {
              ia++;
              Phi_Phi->data[ic] += Phi->data[ib] * a->data[ia];
            }
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  /* 'mpc_gain:37' Phi_F= Phi'*F; */
  i0 = a->size[0] * a->size[1];
  a->size[0] = Phi->size[1];
  a->size[1] = Phi->size[0];
  emxEnsureCapacity((emxArray__common *)a, i0, (int)sizeof(double));
  cr = Phi->size[0];
  for (i0 = 0; i0 < cr; i0++) {
    kk = Phi->size[1];
    for (i1 = 0; i1 < kk; i1++) {
      a->data[i1 + a->size[0] * i0] = Phi->data[i0 + Phi->size[0] * i1];
    }
  }

  if ((a->size[1] == 1) || (F->size[0] == 1)) {
    i0 = Phi_F->size[0] * Phi_F->size[1];
    Phi_F->size[0] = a->size[0];
    Phi_F->size[1] = 6;
    emxEnsureCapacity((emxArray__common *)Phi_F, i0, (int)sizeof(double));
    cr = a->size[0];
    for (i0 = 0; i0 < cr; i0++) {
      for (i1 = 0; i1 < 6; i1++) {
        Phi_F->data[i0 + Phi_F->size[0] * i1] = 0.0;
        kk = a->size[1];
        for (i2 = 0; i2 < kk; i2++) {
          Phi_F->data[i0 + Phi_F->size[0] * i1] += a->data[i0 + a->size[0] * i2]
            * F->data[i2 + F->size[0] * i1];
        }
      }
    }
  } else {
    k = a->size[1];
    unnamed_idx_0 = (unsigned int)a->size[0];
    m = a->size[0];
    i0 = Phi_F->size[0] * Phi_F->size[1];
    Phi_F->size[0] = (int)unnamed_idx_0;
    Phi_F->size[1] = 6;
    emxEnsureCapacity((emxArray__common *)Phi_F, i0, (int)sizeof(double));
    cr = (int)unnamed_idx_0 * 6;
    for (i0 = 0; i0 < cr; i0++) {
      Phi_F->data[i0] = 0.0;
    }

    if (a->size[0] == 0) {
    } else {
      c = a->size[0] * 5;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        i0 = cr + m;
        for (ic = cr; ic + 1 <= i0; ic++) {
          Phi_F->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= c)) {
        ar = -1;
        i0 = br + k;
        for (ib = br; ib + 1 <= i0; ib++) {
          if (F->data[ib] != 0.0) {
            ia = ar;
            i1 = cr + m;
            for (ic = cr; ic + 1 <= i1; ic++) {
              ia++;
              Phi_F->data[ic] += F->data[ib] * a->data[ia];
            }
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  /* 'mpc_gain:38' Phi_R=Phi'*BarRs; */
  i0 = a->size[0] * a->size[1];
  a->size[0] = Phi->size[1];
  a->size[1] = Phi->size[0];
  emxEnsureCapacity((emxArray__common *)a, i0, (int)sizeof(double));
  cr = Phi->size[0];
  for (i0 = 0; i0 < cr; i0++) {
    kk = Phi->size[1];
    for (i1 = 0; i1 < kk; i1++) {
      a->data[i1 + a->size[0] * i0] = Phi->data[i0 + Phi->size[0] * i1];
    }
  }

  if ((a->size[1] == 1) || ((int)(Np * 2.0F) == 1)) {
    i0 = Phi_R->size[0] * Phi_R->size[1];
    Phi_R->size[0] = a->size[0];
    Phi_R->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)Phi_R, i0, (int)sizeof(double));
    cr = a->size[0];
    for (i0 = 0; i0 < cr; i0++) {
      for (i1 = 0; i1 < 2; i1++) {
        Phi_R->data[i0 + Phi_R->size[0] * i1] = 0.0;
        kk = a->size[1];
        for (i2 = 0; i2 < kk; i2++) {
          Phi_R->data[i0 + Phi_R->size[0] * i1] += a->data[i0 + a->size[0] * i2];
        }
      }
    }
  } else {
    k = a->size[1];
    unnamed_idx_0 = (unsigned int)a->size[0];
    m = a->size[0];
    i0 = Phi_R->size[0] * Phi_R->size[1];
    Phi_R->size[0] = (int)unnamed_idx_0;
    Phi_R->size[1] = 2;
    emxEnsureCapacity((emxArray__common *)Phi_R, i0, (int)sizeof(double));
    cr = (int)unnamed_idx_0 << 1;
    for (i0 = 0; i0 < cr; i0++) {
      Phi_R->data[i0] = 0.0;
    }

    if (a->size[0] == 0) {
    } else {
      cr = 0;
      while ((m > 0) && (cr <= m)) {
        i0 = cr + m;
        for (ic = cr; ic + 1 <= i0; ic++) {
          Phi_R->data[ic] = 0.0;
        }

        cr += m;
      }

      br = 0;
      cr = 0;
      while ((m > 0) && (cr <= m)) {
        ar = -1;
        i0 = br + k;
        for (ib = br + 1; ib <= i0; ib++) {
          ia = ar;
          i1 = cr + m;
          for (ic = cr; ic + 1 <= i1; ic++) {
            ia++;
            Phi_R->data[ic] += a->data[ia];
          }

          ar += m;
        }

        br += k;
        cr += m;
      }
    }
  }

  emxFree_real_T(&a);
}

/*
 * File trailer for get_b_constraints.c
 *
 * [EOF]
 */
