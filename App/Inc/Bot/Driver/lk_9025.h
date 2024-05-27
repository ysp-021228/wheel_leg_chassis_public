#ifndef WHEEL_LEG_APP_INC_BOT_DRIVER_LK_9025_H_
#define WHEEL_LEG_APP_INC_BOT_DRIVER_LK_9025_H_

#include "stdint-gcc.h"
#include "can_device.h"

struct Lk9025 {
  uint32_t id;
  float angular_vel;
  float torque;
  uint32_t last_heartbeat_timestamp_ms;
};

void lk9025_init(struct Lk9025 *motor, uint32_t device_id);

void lk9025_set_enable(enum CanType can_type, enum Lk9025SendID CMD_ID);

void lk9025_torque_set(enum CanType can_type, enum Lk9025SendID CMD_ID, float motor_torque);

void lk9025_multi_torque_set(enum CanType can_type, float motor1_torque, float motor2_torque);

void lk9025_clear_motor_errors(enum CanType can_type, enum Lk9025SendID CMD_ID);

void lk9025_can_msg_unpack(uint32_t id, uint8_t data[]);

#endif //WHEEL_LEG_APP_INC_BOT_DRIVER_LK_9025_H_
