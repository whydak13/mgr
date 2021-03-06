/*
 * File: QPhild2.c
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 06-Aug-2017 14:21:28
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
 * Arguments    : const double H[9]
 *                const double f[3]
 *                const double A_cons[36]
 *                const double b[12]
 *                double eta[3]
 * Return Type  : void
 */
void QPhild2(const double H[9], const double f[3], const double A_cons[36],
             const double b[12], double eta[3])
{
  double b_H[9];
  int i3;
  double kk;
  int i;
  double d2;
  double b_A_cons[36];
  double b_b[36];
  double P[144];
  int km;
  double c_b[3];
  double d[12];
  double lambda[12];
  boolean_T exitg1;
  double lambda_p[12];

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
    d2 = 0.0;
    for (i3 = 0; i3 < 3; i3++) {
      d2 += A_cons[i + 12 * i3] * eta[i3];
    }

    if (d2 > b[i]) {
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
        P[i3 + 12 * i] = 0.0;
        for (km = 0; km < 3; km++) {
          P[i3 + 12 * i] += A_cons[i3 + 12 * km] * b_b[km + 3 * i];
        }
      }
    }

    /* 'QPhild2:22' d=(A_cons*(H\f)+b); */
    mldivide(H, f, c_b);
    for (i3 = 0; i3 < 12; i3++) {
      d2 = 0.0;
      for (i = 0; i < 3; i++) {
        d2 += A_cons[i3 + 12 * i] * c_b[i];
      }

      d[i3] = d2 + b[i3];
    }

    /* 'QPhild2:23' [n,m]=size(d); */
    /* 'QPhild2:24' x_ini=zeros(n,m); */
    /* 'QPhild2:25' lambda=x_ini; */
    memset(&lambda[0], 0, 12U * sizeof(double));

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
        /* 'QPhild2:36' lambda(i,1)=max(0,la); */
        d2 = 0.0;
        for (i3 = 0; i3 < 12; i3++) {
          d2 += P[i + 12 * i3] * lambda[i3];
        }

        lambda[i] = fmax(0.0, -((d2 - P[i + 12 * i] * lambda[i]) + d[i]) / P[i +
                         12 * i]);
      }

      /* 'QPhild2:38' al=(lambda-lambda_p)'*(lambda-lambda_p); */
      /* 'QPhild2:39' if (al<10e-8) */
      d2 = 0.0;
      for (i3 = 0; i3 < 12; i3++) {
        d2 += (lambda[i3] - lambda_p[i3]) * (lambda[i3] - lambda_p[i3]);
      }

      if (d2 < 1.0E-7) {
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
      d2 = 0.0;
      for (i = 0; i < 12; i++) {
        d2 += b_b[i3 + 3 * i] * lambda[i];
      }

      eta[i3] = c_b[i3] - d2;
    }
  }
}

/*
 * File trailer for QPhild2.c
 *
 * [EOF]
 */
