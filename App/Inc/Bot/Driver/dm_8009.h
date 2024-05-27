#ifndef DM_6006_H
#define DM_6006_H

#include "stdint-gcc.h"
#include "can_device.h"

struct Dm8009 {
  uint32_t id;
  float pos_r;
  float angular_vel;
  float torque;
  uint32_t last_heartbeat_timestamp_ms;
};

void dm8009_init(struct Dm8009 *motor, uint32_t device_id);

void set_dm8009_torque(enum CanType can_type,
                       enum Dm8009SendID CMD_ID,
                       float torque);

void set_dm8009_MIT(enum CanType can_type, enum Dm8009SendID CMD_ID, float pos, float speed, float kp, float kd, float torque);

void set_dm8009_enable(enum CanType can_type, enum Dm8009SendID CMD_ID);

void set_dm8009_disable(enum CanType can_type, enum Dm8009SendID CMD_ID);

void set_dm8009_pos_speed(enum CanType can_type, enum Dm8009SendID CMD_ID, float pos_rad, float speed_rps);

void dm8009_can_msg_unpack(uint32_t id, uint8_t data[]);

#endif //DM_6006_H
