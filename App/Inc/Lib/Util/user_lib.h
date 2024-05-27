#ifndef USER_LIB_H
#define USER_LIB_H

#include "main.h"
#include "stdint-gcc.h"

//typedef  struct
//{
//    fp32 input;        //��������
//    fp32 out;          //�������
//    fp32 min_value;    //�޷���Сֵ
//    fp32 max_value;    //�޷����ֵ
//    fp32 frame_period; //ʱ����
//}__packed ramp_function_source_t;

typedef  struct
{
  float input;        //��������
  float out;          //�˲����������
  float num;       //�˲�����
  float frame_period; //�˲���ʱ���� ��λ s
}__packed first_order_filter_type_t;

//���ٿ���
extern float invSqrt(float num);
extern float my_sqrt(float number);
extern float my_pow(float number);
//б��������ʼ��
//void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//б����������
//void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
//һ���˲���ʼ��
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period,  float num);
//һ���˲�����
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input);
//��������
//�жϷ���λ
extern float sign(float value);
//��������
extern float fp32_deadline(float Value, float minValue, float maxValue);
//int26����
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
extern float fp32_constrain(float Value, float minValue, float maxValue);
//�޷�����
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//ѭ���޷�����
extern float loop_fp32_constrain(float Input, float minValue, float maxValue);
//�Ƕ� ���޷� 180 ~ -180
extern float theta_format(float Ang);

//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#define VAL_LIMIT(val, min, max) \
  if ((val) <= (min)) {          \
    (val) = (min);               \
  } else if ((val) >= (max)) {   \
    (val) = (max);               \
  }

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


#define RADIAN_COEF  57.3f //�Ƕ�ת����
#ifndef PI
#define PI  3.14159265358979f
#endif

#ifndef ABS
#define ABS(x) ( (x)>0?(x):(-x) )
#endif

#define GRAVITY_A 9.8f


#endif //USER_LIB_H
