/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: transfC_initialize.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 13-Feb-2023 17:14:13
 */

/* Include Files */
#include "transfC_initialize.h"
#include "rt_nonfinite.h"
#include "transfC.h"
#include "transfC_data.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void transfC_initialize(void)
{
  rt_InitInfAndNaN();
  transfC_init();
  isInitialized_transfC = true;
}

/*
 * File trailer for transfC_initialize.c
 *
 * [EOF]
 */
