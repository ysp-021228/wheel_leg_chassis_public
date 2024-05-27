#ifndef WHEEL_LEG_APP_INC_LIB_FILTER_VX_KALMAN_FILTER_H_
#define WHEEL_LEG_APP_INC_LIB_FILTER_VX_KALMAN_FILTER_H_


#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#ifndef CC_ARM
#define CC_ARM
#endif
#ifndef ARM_MATH_MATRIX_CHECK
#define ARM_MATH_MATRIX_CHECK
#endif
#ifndef ARM_MATH_ROUNDING
#define ARM_MATH_ROUNDING
#endif

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"
#include "arm_math.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

#define mat arm_matrix_instance_f32
#define Matrix_Init arm_mat_init_f32
#define Matrix_Add arm_mat_add_f32
#define Matrix_Subtract arm_mat_sub_f32
#define Matrix_Multiply arm_mat_mult_f32
#define Matrix_Transpose arm_mat_trans_f32
#define Matrix_Inverse arm_mat_inverse_f32

struct KalmanFilter{
  //����
//    float dt;
  uint8_t xSize;
  uint8_t zSize;
  uint8_t ySize;
  uint8_t uSize;
//    uint16_t measureSize;
  //����
  mat H;//�۲����
  mat F;//״̬ת�ƾ���
  mat R;//measure noise
  mat Q;//process noise
  mat Xk;//�˲����²���ǰʱ�����
  mat Xk_1;//�˲�Ԥ�ⲽ���
  mat K;//����������
  mat sigma;
  mat sigma_minus;
  mat tempmat;//�����������ʱ�洢����
  mat tempmat1;
  mat tempmat2;
  mat tempmat3;
  mat tempmat4;
  mat tempmat5;
  mat tempmat6;
  mat tempmat7;
  mat tempmat8;
  mat FT;//F��ת��
  mat HT;//H��ת��
//    mat measure;
  mat E;//��λ��
  //���������,��������
  float* H_data;
  float* F_data;
  float* R_data;
  float* Q_data;
  float* Xk_data;
  float* Xk_1_data;
  float* k_data;
  float* sigma_data;
  float* sigma_minus_data;
  float* tempmat_data;
  float* tempmat1_data;
  float* tempmat2_data;
  float* tempmat3_data;
  float* tempmat4_data;
  float* tempmat5_data;
  float* tempmat6_data;
  float* tempmat7_data;
  float* tempmat8_data;
  float* FT_data;
  float* HT_data;
  float* restrict_Variance;
//    float* measure_data;
  float* E_data;
};
extern uint16_t sizeof_float;
void kalman_Filter_Init(struct KalmanFilter* kf, uint8_t xSize, uint8_t uSize,
                        uint8_t zSize, uint8_t ySize);
float* kalman_Filter_Predict(struct KalmanFilter* kf);
float* kalman_Filter_Update(struct KalmanFilter* kf, float* measure_temp);

#endif //WHEEL_LEG_APP_INC_LIB_FILTER_VX_KALMAN_FILTER_H_
