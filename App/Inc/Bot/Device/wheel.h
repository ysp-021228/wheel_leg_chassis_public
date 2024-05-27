#ifndef WHEEL_H
#define WHEEL_H

#include "can_device.h"
#include "lk_9025.h"

void wheel_init();

void set_wheel_torque(float torque_nm_L, float torque_nm_R);

struct Lk9025 *get_wheel_motors();

#endif //WHEEL_H
