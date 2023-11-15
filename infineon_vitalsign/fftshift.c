/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: fftshift.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 13-Feb-2023 17:14:13
 */

/* Include Files */
#include "fftshift.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : creal32_T x[256]
 * Return Type  : void
 */
void fftshift(creal32_T x[256])
{
  float xtmp_im;
  float xtmp_re;
  int a;
  int dim;
  int i1;
  int ib;
  int ic;
  int j;
  int k;
  int midoffset;
  int vlend2;
  int vstride;
  for (dim = 0; dim < 2; dim++) {
    ic = dim - 1;
    if (dim + 1 <= 1) {
      a = 256;
    } else {
      a = 1;
    }
    if (a > 1) {
      vlend2 = a / 2;
      vstride = 1;
      for (k = 0; k <= ic; k++) {
        vstride <<= 8;
      }
      midoffset = vlend2 * vstride - 1;
      if (vlend2 << 1 == a) {
        i1 = 0;
        for (j = 0; j < vstride; j++) {
          i1++;
          ib = i1 + midoffset;
          for (k = 0; k < vlend2; k++) {
            a = k * vstride;
            ic = (i1 + a) - 1;
            xtmp_re = x[ic].re;
            xtmp_im = x[ic].im;
            a += ib;
            x[ic] = x[a];
            x[a].re = xtmp_re;
            x[a].im = xtmp_im;
          }
        }
      } else {
        i1 = 0;
        for (j = 0; j < vstride; j++) {
          i1++;
          ib = i1 + midoffset;
          xtmp_re = x[ib].re;
          xtmp_im = x[ib].im;
          for (k = 0; k < vlend2; k++) {
            ic = ib + vstride;
            a = (i1 + k * vstride) - 1;
            x[ib] = x[a];
            x[a] = x[ic];
            ib = ic;
          }
          x[ib].re = xtmp_re;
          x[ib].im = xtmp_im;
        }
      }
    }
  }
}

/*
 * File trailer for fftshift.c
 *
 * [EOF]
 */
