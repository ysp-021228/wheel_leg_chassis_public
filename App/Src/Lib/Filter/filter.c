#include "filter.h"

void init_filter(struct MovingAverageFilter *filter) {
  // �����������е�Ԫ�س�ʼ��Ϊ0
  for (int i = 0; i < WINDOW_SIZE; i++) {
    filter->window[i] = 0;
  }

  // ��ʼ����������
  filter->index = 0;
  filter->sum = 0;
}

// �����˲���״̬
void updateFilter(struct MovingAverageFilter *filter, float newValue) {
  // ��ȥ��ֵ
  filter->sum -= filter->window[filter->index];

  // �����ֵ
  filter->window[filter->index] = newValue;
  filter->sum += newValue;

  // ��������
  filter->index = (filter->index + 1) % WINDOW_SIZE;
}

float getFilteredValue(const struct MovingAverageFilter *filter) {
  // ����ƽ��ֵ
  return filter->sum / WINDOW_SIZE;
}