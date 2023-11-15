/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: filter.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 13-Feb-2023 17:14:13
 */

/* Include Files */
#include "filter.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <string.h>

/* Function Definitions */
/*
 * Arguments    : double b[5]
 *                double a[5]
 *                const float x[256]
 *                float y[256]
 * Return Type  : void
 */
void filter(double b[5], double a[5], const float x[256], float y[256])
{
  double a1;
  float as;
  int j;
  int k;
  int naxpy;
  int y_tmp;
  a1 = a[0];
  if ((!rtIsInf(a[0])) && (!rtIsNaN(a[0])) && (!(a[0] == 0.0)) &&
      (a[0] != 1.0)) {
    for (k = 0; k < 5; k++) {
      b[k] /= a1;
    }
    a[1] /= a[0];
    a[2] /= a[0];
    a[3] /= a[0];
    a[4] /= a[0];
    a[0] = 1.0;
  }
  memset(&y[0], 0, 256U * sizeof(float));
  for (k = 0; k < 256; k++) {
    if (256 - k < 5) {
      naxpy = 255 - k;
    } else {
      naxpy = 4;
    }
    for (j = 0; j <= naxpy; j++) {
      y_tmp = k + j;
      y[y_tmp] += x[k] * (float)b[j];
    }
    if (255 - k < 4) {
      naxpy = 254 - k;
    } else {
      naxpy = 3;
    }
    as = -y[k];
    for (j = 0; j <= naxpy; j++) {
      y_tmp = (k + j) + 1;
      y[y_tmp] += as * (float)a[j + 1];
    }
  }
}

/*
 * File trailer for filter.c
 *
 * [EOF]
 */
