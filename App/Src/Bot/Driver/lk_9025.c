#include "lk_9025.h"
#include "can_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "user_lib.h"

#define LK_TORQUE_CONSTANT 0.32f
#define LK_CURRENT_2_DATA 62.5f

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static unsigned char is_init = 0;
static uint8_t motors_len = 0;
static struct Lk9025 *motors[3];
static CAN_TxHeaderTypeDef
    tx_msg = {0x00, 0, CAN_ID_STD, CAN_RTR_DATA, 0x08, DISABLE};
static CAN_RxHeaderTypeDef rx_msg;

static uint8_t rx_data[9];

static void lk9025_register(struct Lk9025 *motor) {
  motors[motors_len] = motor;
  ++motors_len;
}

void lk9025_init(struct Lk9025 *motor, uint32_t device_id) {
  motor->id = device_id;

  motor->torque = 0;
  motor->angular_vel = 0;

  motor->last_heartbeat_timestamp_ms = 0;

  lk9025_register(motor);
}

void lk9025_set_enable(enum CanType can_type, enum Lk9025SendID CMD_ID) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  uint8_t tx_data[8] = {0};
  tx_data[0] = 0x88;
  tx_data[1] = 0;
  tx_data[2] = 0;
  tx_data[3] = 0;
  tx_data[4] = 0;
  tx_data[5] = 0;
  tx_data[6] = 0;
  tx_data[7] = 0;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void lk9025_torque_set(enum CanType can_type, enum Lk9025SendID CMD_ID, float motor_torque) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  float motor_current;
  int16_t motor_data;

  motor_current = motor_torque / LK_TORQUE_CONSTANT;
  motor_data = motor_current * LK_CURRENT_2_DATA;

  uint8_t tx_data[8] = {0};
  tx_data[0] = 0xA1;
  tx_data[1] = 0;
  tx_data[2] = 0;
  tx_data[3] = 0;
  tx_data[4] = *(uint8_t *) (&motor_data);
  tx_data[5] = *((uint8_t *) (&motor_data) + 1);
//  tx_data[4] = motor_data << 8;
//  tx_data[5] = motor_data >> 8;
  tx_data[6] = 0;
  tx_data[7] = 0;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void lk9025_multi_torque_set(enum CanType can_type, float motor1_torque, float motor2_torque) {
  tx_msg.StdId = 0x280;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  float motor1_current, motor2_current;
  int16_t motor1_data, motor2_data;
  motor1_current = motor1_torque / LK_TORQUE_CONSTANT;
  motor2_current = motor2_torque / LK_TORQUE_CONSTANT;

  motor1_data = motor1_current * LK_CURRENT_2_DATA;
  motor2_data = motor2_current * LK_CURRENT_2_DATA;

  uint8_t tx_data[8] = {0};
//  tx_data[0] = *(uint8_t *) (&motor1_data);
//  tx_data[1] = *((uint8_t *) (&motor1_data) + 1);
//  tx_data[2] = *(uint8_t *) (&motor2_data);
//  tx_data[3] = *((uint8_t *) (&motor2_data) + 1);
  tx_data[0] = motor1_data << 8;
  tx_data[1] = motor1_data >> 8;
  tx_data[2] = motor2_data << 8;
  tx_data[3] = motor2_data >> 8;
  tx_data[4] = 0;
  tx_data[5] = 0;
  tx_data[6] = 0;
  tx_data[7] = 0;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void lk9025_clear_motor_errors(enum CanType can_type, enum Lk9025SendID CMD_ID) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  uint8_t tx_data[8] = {0};
  tx_data[0] = 0x9B;
  tx_data[1] = 0;
  tx_data[2] = 0;
  tx_data[3] = 0;
  tx_data[4] = 0;
  tx_data[5] = 0;
  tx_data[6] = 0;
  tx_data[7] = 0;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void lk9025_can_msg_unpack(uint32_t id, uint8_t data[]) {
  int16_t speed_int, iq_int;
  switch (id) {
    case WHEEL_L_RECEIVE: {
      speed_int = (int16_t) ((data)[5] << 8 | (data)[4]);
      iq_int = (int16_t) ((data)[3] << 8 | (data)[2]);
      motors[0]->angular_vel = speed_int * PI / 180;
      motors[0]->torque = (iq_int / LK_CURRENT_2_DATA) * LK_TORQUE_CONSTANT;
      break;
    }

    case WHEEL_R_RECEIVE: {
      speed_int = (int16_t) ((data)[5] << 8 | (data)[4]);
      iq_int = (int16_t) ((data)[3] << 8 | (data)[2]);
      motors[1]->angular_vel = speed_int * PI / 180;
      motors[1]->torque = (iq_int / LK_CURRENT_2_DATA) * LK_TORQUE_CONSTANT;
      break;
    }
    default:break;
  }
}

