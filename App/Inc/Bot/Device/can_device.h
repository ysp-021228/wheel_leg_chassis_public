#ifndef CAN_DEVICE_H
#define CAN_DEVICE_H

#include "stdint-gcc.h"

enum CanType {
  CAN_1,
  CAN_2,
};

enum Dm8009SendID {
  JOINT_LF_SEND = 0x10,
  JOINT_LB_SEND = 0x11,
  JOINT_RF_SEND = 0x12,
  JOINT_RB_SEND = 0x13,
};

enum Lk9025SendID {
  WHEEL_L_SEND = 0x141,
  WHEEL_R_SEND = 0x142,
};

enum GimbalSendID {
  CHASSIS_ANGLE_VEL_INFO = 0x111,
};

enum CanReceiveDeviceId {
  WHEEL_L_RECEIVE = 0x141,
  WHEEL_R_RECEIVE = 0x142,
  JOINT_LF_RECEIVE = 0x00,
  JOINT_LB_RECEIVE = 0x01,
  JOINT_RF_RECEIVE = 0x02,
  JOINT_RB_RECEIVE = 0x03,
  CHASSIS_MODE_HEIGHT_INFO = 0x101,
  CHASSIS_SPEED_INFO = 0x102,
  CHASSIS_ANGLE_INFO = 0x103,
};

void can_filter_init(void);

uint32_t get_can1_free_mailbox();
uint32_t get_can2_free_mailbox();

#endif //CAN_DEVICE_H
