#include "filter.h"

void init_filter(struct MovingAverageFilter *filter) {
  // 将窗口数组中的元素初始化为0
  for (int i = 0; i < WINDOW_SIZE; i++) {
    filter->window[i] = 0;
  }

  // 初始化其他变量
  filter->index = 0;
  filter->sum = 0;
}

// 更新滤波器状态
void updateFilter(struct MovingAverageFilter *filter, float newValue) {
  // 减去旧值
  filter->sum -= filter->window[filter->index];

  // 添加新值
  filter->window[filter->index] = newValue;
  filter->sum += newValue;

  // 更新索引
  filter->index = (filter->index + 1) % WINDOW_SIZE;
}

float getFilteredValue(const struct MovingAverageFilter *filter) {
  // 计算平均值
  return filter->sum / WINDOW_SIZE;
}