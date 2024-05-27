#ifndef WHEEL_LEG_APP_INC_BOT_SUBSYSTEM_GIMBAL_COMMUNICATION_H_
#define WHEEL_LEG_APP_INC_BOT_SUBSYSTEM_GIMBAL_COMMUNICATION_H_

#include <stdint-gcc.h>
#include "chassis.h"

struct GimbalMsg {
  enum ChassisCtrlMode chassis_ctrl_mode;
  struct ChassisCtrlInfo chassis_ctrl_info;
};

void gimbal_msg_unpack(uint32_t id, uint8_t data[]);

struct GimbalMsg *get_gimbal_msg();

extern void gimbal_task(void const *pvParameters);

#endif //WHEEL_LEG_APP_INC_BOT_SUBSYSTEM_GIMBAL_COMMUNICATION_H_
