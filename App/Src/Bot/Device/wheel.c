#include "wheel.h"
#include "lk_9025.h"
#include "cmsis_os.h"

static struct Lk9025 wheel[2];

void wheel_init() {
  lk9025_init(&wheel[0], WHEEL_L_RECEIVE);
  lk9025_init(&wheel[1], WHEEL_R_RECEIVE);
  lk9025_set_enable(CAN_2,WHEEL_L_SEND);
  lk9025_set_enable(CAN_2,WHEEL_R_SEND);
}

void set_wheel_torque(float torque_nm_L, float torque_nm_R) {
  lk9025_torque_set(CAN_2,WHEEL_L_SEND,torque_nm_L);
  osDelay(1);
  lk9025_torque_set(CAN_2,WHEEL_R_SEND,torque_nm_R);
}

struct Lk9025 *get_wheel_motors(){
  return wheel;
}