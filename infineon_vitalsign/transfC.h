/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: transfC.h
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 13-Feb-2023 17:14:13
 */

#ifndef TRANSFC_H
#define TRANSFC_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void transfC(const unsigned short adcDataRow[128], float *breath_count,
                    float *heart_count);

void transfC_init(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for transfC.h
 *
 * [EOF]
 */
