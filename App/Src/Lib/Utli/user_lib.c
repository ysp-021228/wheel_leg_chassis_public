#include "user_lib.h"
#include "arm_math.h"

//���ٿ���
float invSqrt(float num)
{
  float halfnum = 0.5f * num;
  float y = num;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfnum * y * y));
  return y;
}

//����ƽ�����㷨
//��������invSqrt(void)
//��������ƽ�����ĵ���
//�ú����Ǿ����Carmack��ƽ�����㷨��Ч�ʼ��ߣ�ʹ��ħ��0x5f375a86
float my_sqrt(float number)
{
  long i;
  float xx, yy;
  const float f = 1.5F;
  xx = number * 0.5F;
  yy = number;
  i = * ( long * ) &yy;
  i = 0x5f375a86 - ( i >> 1 );

  yy = * ( float * ) &i;
  yy = yy * ( f - ( xx * yy * yy ) );
  yy = yy * ( f - ( xx * yy * yy ) );
  return number * yy;
}

//���㸡����ƽ��
float my_pow(float a)
{
  return a*a;
}
/**
  * @brief          б��������ʼ��
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      ���ֵ
  * @param[in]      ��Сֵ
  * @retval         ���ؿ�
  */
//void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min)
//{
//    ramp_source_type->frame_period = frame_period;
//    ramp_source_type->max_value = max;
//    ramp_source_type->min_value = min;
//    ramp_source_type->input = 0.0f;
//    ramp_source_type->out = 0.0f;
//}

/**
  * @brief          б���������㣬���������ֵ���е��ӣ� ���뵥λΪ /s ��һ������������ֵ
  * @author         RM
  * @param[in]      б�������ṹ��
  * @param[in]      ����ֵ
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
//void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input)
//{
//    ramp_source_type->input = input;
//    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
//    if (ramp_source_type->out > ramp_source_type->max_value)
//    {
//        ramp_source_type->out = ramp_source_type->max_value;
//    }
//    else if (ramp_source_type->out < ramp_source_type->min_value)
//    {
//        ramp_source_type->out = ramp_source_type->min_value;
//    }
//}
/**
  * @brief          һ�׵�ͨ�˲���ʼ��
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period,  float num)
{
  first_order_filter_type->frame_period = frame_period;
  first_order_filter_type->num = num;
  first_order_filter_type->input = 0.0f;
  first_order_filter_type->out = 0.0f;
}

/**
  * @brief          һ�׵�ͨ�˲�����
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @retval         ���ؿ�
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
  first_order_filter_type->input = input;
  first_order_filter_type->out =
      first_order_filter_type->num / (first_order_filter_type->num
          + first_order_filter_type->frame_period) * first_order_filter_type->out
          + first_order_filter_type->frame_period / (first_order_filter_type->num
              + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//��������
void abs_limit(float *num, float Limit)
{
  if (*num > Limit)
  {
    *num = Limit;
  }
  else if (*num < -Limit)
  {
    *num = -Limit;
  }
}

//�жϷ���λ
float sign(float value)
{
  if (value >= 0.0f)
  {
    return 1.0f;
  }
  else
  {
    return -1.0f;
  }
}

//��������
float fp32_deadline(float Value, float minValue, float maxValue)
{
  if (Value < maxValue && Value > minValue)
  {
    Value = 0.0f;
  }
  return Value;
}

//int26����
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
  if (Value < maxValue && Value > minValue)
  {
    Value = 0;
  }
  return Value;
}

//�޷�����
float fp32_constrain(float Value, float minValue, float maxValue)
{
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

//�޷�����
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

//ѭ���޷�����
float loop_fp32_constrain(float Input, float minValue, float maxValue)
{
  if (maxValue < minValue)
  {
    return Input;
  }

  if (Input > maxValue)
  {
    float len = maxValue - minValue;
    while (Input > maxValue)
    {
      Input -= len;
    }
  }
  else if (Input < minValue)
  {
    float len = maxValue - minValue;
    while (Input < minValue)
    {
      Input += len;
    }
  }
  return Input;
}

//���ȸ�ʽ��Ϊ-PI~PI

//�Ƕȸ�ʽ��Ϊ-180~180
float theta_format(float Ang)
{
  return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}
