#ifndef LQR_H
#define LQR_H
#include "filter.h"
struct Chassis;
extern struct MovingAverageFilter theta_ddot_filter_L,theta_ddot_filter_R;
void lqr_ctrl(struct Chassis* chassis);

#endif //LQR_H
