/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: transfC.c
 *
 * MATLAB Coder version            : 5.2
 * C/C++ source code generated on  : 13-Feb-2023 17:14:13
 */

/* Include Files */
#include "transfC.h"
#include "FFTImplementationCallback.h"
#include "fftshift.h"
#include "filter.h"
#include "rt_nonfinite.h"
#include "transfC_data.h"
#include "transfC_initialize.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>
#include "arm_math.h" 
#include "usbd_cdc_if.h"

/* Variable Definitions */
static float phaseBuffer[256];

/* Function Declarations */
static float rt_atan2f_snf(float u0, float u1);

static float rt_hypotf_snf(float u0, float u1);

/* Function Definitions */
/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_atan2f_snf(float u0, float u1)
{
  float y;
  int b_u0;
  int b_u1;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = rtNaNF;
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    if (u0 > 0.0F) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }
    if (u1 > 0.0F) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }
    y = (float)atan2((float)b_u0, (float)b_u1);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (float)atan2(u0, u1);
  }
  return y;
}

/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_hypotf_snf(float u0, float u1)
{
  float a;
  float y;
  a = (float)fabs(u0);
  y = (float)fabs(u1);
  if (a < y) {
    a /= y;
    y *= (float)sqrt(a * a + 1.0F);
  } else if (a > y) {
    y /= a;
    y = a * (float)sqrt(y * y + 1.0F);
  } else if (!rtIsNaNF(y)) {
    y = a * 1.41421354F;
  }
  return y;
}

/*
 * tic;
 *
 * Arguments    : const unsigned short adcDataRow[128]
 *                float *breath_count
 *                float *heart_count
 * Return Type  : void
 */
void transfC(const unsigned short adcDataRow[128], float *breath_count,
             float *heart_count)
{
  static const double dv1[5] = {0.0019013594933556097, 0.0,
                                -0.0038027189867112193, 0.0,
                                0.0019013594933556097};
  static const double dv3[5] = {1.0, -3.9018233021208659, 5.7262803454085951,
                                -3.7466598739850032, 0.922226212902835};
  static const double dv4[5] = {0.015875914972992084, 0.0,
                                -0.031751829945984168, 0.0,
                                0.015875914972992084};
  static const double dv5[5] = {1.0, -3.4046571522551696, 4.6505685964807721,
                                -3.0100826942817078, 0.78582345738106174};
  creal_T y[128];
  creal32_T b_y[256];
  creal32_T sig_ffta[128];
  float b_adcDataRow[128];
  double dv[5];
  double dv2[5];
  float fft_breath[256];
  float varargin_1[64];
  float breath_fre_max;
  float f;
  int b_i;
  int breath_index;
  int i;
  int i1;
  int i2;
  int i3;
  int idx;
  boolean_T exitg1;
  if (!isInitialized_transfC) {
    transfC_initialize();
  }
  /* 加窗 */
  /* ADC采样点数 */
  /*  fft_data_last=zeros(1,Nr); */
  /*  range_max=0; */
  /*  Buffer */
  /*  FFT */
  for (breath_index = 0; breath_index < 128; breath_index++) {
    b_adcDataRow[breath_index] = adcDataRow[breath_index];
  }
  
  float32_t output[128];
  
  // 初始化FFT状态
    arm_rfft_fast_instance_f32 fft_inst;
    arm_rfft_fast_init_f32(&fft_inst, 128);
    
    // 执行FFT
    arm_rfft_fast_f32(&fft_inst, b_adcDataRow, output,0);
	
  //c_FFTImplementationCallback_doH(b_adcDataRow, 0, y);
  for (i = 0; i < 128; i++) {
	 // usb_printf("%lf\t%lf\t",(float)y[i].re,(float)y[i].im);
	  if(i%2==0){
		sig_ffta[i].re = output[i];
	  }
    
    sig_ffta[i].im = output[i];
  }
  /*  平均相消 */
  /*  rangeProfile = complex(zeros(Nr,numChannels)); */
  /*  for k=1:numChannels */
  /*        rangeProfile(:,k) = sig_ffta(:,k)- temp_mean; */
  /*  end */
  sig_ffta[0].re = 0.0F;
  sig_ffta[0].im = 0.0F;
  sig_ffta[1].re = 0.0F;
  sig_ffta[1].im = 0.0F;
  sig_ffta[2].re = 0.0F;
  sig_ffta[2].im = 0.0F;
  /*  for ii=1:Nd-1                % 滑动对消，少了一个脉冲 */
  /*       sig_ffta(:,ii) = sig_ffta(:,ii+1)-sig_ffta(:,ii); */
  /*  end */
  /*  sig_ffta=sig_ffta.'; */
  /*  fft_data_abs = abs(sig_ffta); */
  /*  real_data = real(sig_ffta); */
  /*  imag_data = imag(sig_ffta); */
  /*   */
  /*  angle_fft=zeros(Nd,Nr); */
  /*  for i = 1:Nd */
  /*      for j = 1:Nr  %对每一个距离点取相位 extract phase */
  /*          angle_fft(i,j) = atan2(imag_data(i,j),real_data(i,j)); */
  /*      end */
  /*  end */
  /* Range-bin tracking 找出能量最大的点，即人体的位置  */
  /*  max_num=0; */
  /*  for j = 1:Nr/2 */
  /*      for i = 1:Nd % 进行非相干积累 */
  /*          fft_data_last(j) = fft_data_last(j) + fft_data_abs(i,j); */
  /*      end */
  /*      if ( fft_data_last(j) > range_max) */
  /*          range_max = fft_data_last(j); */
  /*          max_num = j; */
  /*      end */
  /*  end  */
  /*  取出能量最大点的相位  extract phase from selected range bin */
  /*  angle_fft_last = angle_fft(:,max_num); */
  for (i = 0; i < 64; i++) {
    varargin_1[i] = rt_hypotf_snf(sig_ffta[i].re, sig_ffta[i].im);
  }
  if (!rtIsNaNF(varargin_1[0])) {
    idx = 1;
  } else {
    idx = 0;
    i = 2;
    exitg1 = false;
    while ((!exitg1) && (i < 65)) {
      if (!rtIsNaNF(varargin_1[i - 1])) {
        idx = i;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }
  if (idx == 0) {
    idx = 1;
  } else {
    breath_fre_max = varargin_1[idx - 1];
    breath_index = idx + 1;
    for (i = breath_index; i < 65; i++) {
      f = varargin_1[i - 1];
      if (breath_fre_max < f) {
        breath_fre_max = f;
        idx = i;
      }
    }
  }
  /*  相位解缠绕 */
  /*  angle_fft_last2=unwrap(angle_fft_last); */
  /*  相位差分 */
  /*  angle_fft_last3=complex(zeros(1,Nd)); */
  /*   */
  /*  for i = 1:Nd-1 */
  /*      angle_fft_last3(i) = angle_fft_last2(i+1) - angle_fft_last2(i); */
  /*      angle_fft_last3(Nd)= angle_fft_last2(Nd)- angle_fft_last2(Nd-1); */
  /*  end  */
  breath_index = 1;
  for (i = 0; i < 255; i++) {
    breath_index = i + 1;
    phaseBuffer[i] = phaseBuffer[i + 1];
  }
  phaseBuffer[breath_index] =
      rt_atan2f_snf(sig_ffta[idx - 1].im, sig_ffta[idx - 1].re);
  /*  滤波 */
  /*  呼吸 */
  for (b_i = 0; b_i < 5; b_i++) {
    dv[b_i] = dv1[b_i];
  }
  for (i1 = 0; i1 < 5; i1++) {
    dv2[i1] = dv3[i1];
  }
  filter(dv, dv2, phaseBuffer, fft_breath);
  d_FFTImplementationCallback_doH(fft_breath, 0, b_y);
  fftshift(b_y);
  for (i = 0; i < 256; i++) {
    fft_breath[i] = rt_hypotf_snf(b_y[i].re, b_y[i].im);
  }
  breath_index = 0;
  breath_fre_max = 0.0F;
  /*  呼吸频率 */
  for (i = 0; i < 128; i++) {
    /* 谱峰最大值搜索 */
    f = fft_breath[i];
    if (f > breath_fre_max) {
      breath_fre_max = f;
      if (f < 0.01) {
        /* 幅度置信 判断是否是存在人的呼吸 */
        breath_index = 129;
      } else {
        breath_index = i + 1;
      }
    }
  }
  *breath_count =
      (float)(20.0 * (128.0 - ((double)breath_index - 1.0)) / 256.0 * 60.0);
  /* 呼吸频率解算 */
  /*  心跳 */
  /* IIR */
  for (i2 = 0; i2 < 5; i2++) {
    dv[i2] = dv4[i2];
  }
  for (i3 = 0; i3 < 5; i3++) {
    dv2[i3] = dv5[i3];
  }
  filter(dv, dv2, phaseBuffer, fft_breath);
  d_FFTImplementationCallback_doH(fft_breath, 0, b_y);
  fftshift(b_y);
  for (i = 0; i < 256; i++) {
    fft_breath[i] = rt_hypotf_snf(b_y[i].re, b_y[i].im);
  }
  breath_index = 0;
  breath_fre_max = 0.0F;
  for (i = 0; i < 128; i++) {
    f = fft_breath[i];
    if (f > breath_fre_max) {
      breath_fre_max = f;
      if (f < 0.01) {
        /* 幅度置信 判断是否是存在人的心跳 */
        breath_index = 129;
      } else {
        breath_index = i + 1;
      }
    }
  }
  *heart_count =
      (float)(20.0 * (128.0 - ((double)breath_index - 1.0)) / 256.0 * 60.0);
  /*  toc; */
  /*  disp(['运行时间: ',num2str(toc)]); */
  /*  disp(['呼吸:',num2str(breath_count), ... */
  /*           '心跳:',num2str(heart_count)]); */
}

/*
 * tic;
 *
 * Arguments    : void
 * Return Type  : void
 */
void transfC_init(void)
{
  memset(&phaseBuffer[0], 0, 256U * sizeof(float));
}

/*
 * File trailer for transfC.c
 *
 * [EOF]
 */
