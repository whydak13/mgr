/*
 * File: mpc_gain.c
 *
 * MATLAB Coder version            : 2.8
 * C/C++ source code generated on  : 13-Aug-2017 17:54:11
 */

/* Include Files */
#include "QPhild2.h"
#include "get_f.h"
#include "mpc_gain.h"
#include "get_f_emxutil.h"

/* Function Declarations */
static int div_nzp_s32_floor(int numerator, int denominator);

/* Function Definitions */

/*
 * Arguments    : int numerator
 *                int denominator
 * Return Type  : int
 */
static int div_nzp_s32_floor(int numerator, int denominator)
{
  int quotient;
  unsigned int absNumerator;
  unsigned int absDenominator;
  boolean_T quotientNeedsNegation;
  unsigned int tempAbsQuotient;
  if (numerator >= 0) {
    absNumerator = (unsigned int)numerator;
  } else {
    absNumerator = (unsigned int)-numerator;
  }

  if (denominator >= 0) {
    absDenominator = (unsigned int)denominator;
  } else {
    absDenominator = (unsigned int)-denominator;
  }

  quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
  tempAbsQuotient = absNumerator / absDenominator;
  if (quotientNeedsNegation) {
    absNumerator %= absDenominator;
    if (absNumerator > 0U) {
      tempAbsQuotient++;
    }
  }

  if (quotientNeedsNegation) {
    quotient = -(int)tempAbsQuotient;
  } else {
    quotient = (int)tempAbsQuotient;
  }

  return quotient;
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
                [0] * div_nzp_s32_floor(ia, i1 - i0)) - 1];
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
                [0] * div_nzp_s32_floor(ia, i1 - i0)) - 1];
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
 * File trailer for mpc_gain.c
 *
 * [EOF]
 */
