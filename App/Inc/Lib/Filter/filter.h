#ifndef FILTER_H
#define FILTER_H

#define WINDOW_SIZE 20

struct MovingAverageFilter {
  float window[WINDOW_SIZE];
  int index;
  float sum;
};
void init_filter(struct MovingAverageFilter *filter);
void updateFilter(struct MovingAverageFilter *filter, float newValue);
float getFilteredValue(const struct MovingAverageFilter *filter);

#endif //FILTER_H
