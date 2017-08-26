/*
 * File: QPhild2.c
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
    while ((!exitg1) && (km < 38)) {
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
 * File trailer for QPhild2.c
 *
 * [EOF]
 */
