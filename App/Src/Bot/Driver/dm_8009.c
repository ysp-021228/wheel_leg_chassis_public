#include "dm_8009.h"
#include "can_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static unsigned char is_init = 0;
static uint8_t motors_len = 0;
static struct Dm8009 *motors[4];
static CAN_TxHeaderTypeDef
    tx_msg = {0x00, 0, CAN_ID_STD, CAN_RTR_DATA, 0x08, DISABLE};
static CAN_RxHeaderTypeDef rx_msg;

static uint8_t rx_data[9];

static void dm6006_register(struct Dm8009 *motor) {
  motors[motors_len] = motor;
  ++motors_len;
}

static int float_to_uint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits///
  float span = x_max - x_min;
  float offset = x_min;
  return (int) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
/// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

void dm8009_init(struct Dm8009 *motor, uint32_t device_id) {
  motor->id = device_id;

  motor->torque = 0;
  motor->angular_vel = 0;
  motor->pos_r = 0;

  motor->last_heartbeat_timestamp_ms = 0;

  dm6006_register(motor);
}

void set_dm8009_enable(enum CanType can_type, enum Dm8009SendID CMD_ID) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  uint8_t tx_data[8] = {0};
  tx_data[0] = 0xFF;
  tx_data[1] = 0xFF;
  tx_data[2] = 0xFF;
  tx_data[3] = 0xFF;
  tx_data[4] = 0xFF;
  tx_data[5] = 0xFF;
  tx_data[6] = 0xFF;
  tx_data[7] = 0xFC;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void set_dm8009_disable(enum CanType can_type, enum Dm8009SendID CMD_ID){
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  uint8_t tx_data[8] = {0};
  tx_data[0] = 0xFF;
  tx_data[1] = 0xFF;
  tx_data[2] = 0xFF;
  tx_data[3] = 0xFF;
  tx_data[4] = 0xFF;
  tx_data[5] = 0xFF;
  tx_data[6] = 0xFF;
  tx_data[7] = 0xFD;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void set_dm8009_MIT(enum CanType can_type,
                    enum Dm8009SendID CMD_ID,
                    float pos,
                    float speed,
                    float kp,
                    float kd,
                    float torque) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  pos_tmp = float_to_uint(pos, -12.5f, 12.5f, 16);
  vel_tmp = float_to_uint(speed, -45, 45, 12);
  kp_tmp = float_to_uint(kp, 0.0f, 500, 12);
  kd_tmp = float_to_uint(kd, 0.0f, 5.0f, 12);
  tor_tmp = float_to_uint(torque, -50, 50, 12);

  uint8_t tx_data[8] = {0};
  tx_data[0] = (pos_tmp >> 8);
  tx_data[1] = pos_tmp;
  tx_data[2] = (vel_tmp >> 4);
  tx_data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  tx_data[4] = kp_tmp;
  tx_data[5] = (kd_tmp >> 4);
  tx_data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  tx_data[7] = tor_tmp;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void set_dm8009_torque(enum CanType can_type,
                       enum Dm8009SendID CMD_ID,
                       float torque) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  uint16_t tor_tmp;
  tor_tmp = float_to_uint(torque, -50, 50, 12);

  uint8_t tx_data[8] = {0};
  tx_data[0] = 0;
  tx_data[1] = 0;
  tx_data[2] = 0;
  tx_data[3] = 0;
  tx_data[4] = 0;
  tx_data[5] = 0;
  tx_data[6] = (tor_tmp >> 8);
  tx_data[7] = tor_tmp;

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void set_dm8009_pos_speed(enum CanType can_type, enum Dm8009SendID CMD_ID, float pos_rad, float speed_rps) {
  tx_msg.StdId = CMD_ID + 0x100;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  uint8_t *posbuf, *speedbuf;
  posbuf = (uint8_t *) &pos_rad;
  speedbuf = (uint8_t *) &speed_rps;

  uint8_t tx_data[8] = {0};
  tx_data[0] = *posbuf;
  tx_data[1] = *(posbuf + 1);
  tx_data[2] = *(posbuf + 2);
  tx_data[3] = *(posbuf + 3);
  tx_data[4] = *speedbuf;
  tx_data[5] = *(speedbuf + 1);
  tx_data[6] = *(speedbuf + 2);
  tx_data[7] = *(speedbuf + 3);

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

void dm8009_can_msg_unpack(uint32_t id, uint8_t data[]) {
  switch (id) {
    case JOINT_LF_RECEIVE: {
      int pos_int, speed_int, torque_int;
      pos_int = (data[1] << 8) | data[2];
      speed_int = (data[3] << 4) | (data[4] >> 4);
      torque_int = (data[4] & 0xF) << 8 | data[5];

      motors[0]->pos_r = uint_to_float(pos_int, -12.5f, 12.5f, 16);
      motors[0]->angular_vel = uint_to_float(speed_int, -45.0f, 45.0f, 12);
      motors[0]->torque = uint_to_float(torque_int, -50.0f, 50.0f, 12);
    }
      break;

    case JOINT_LB_RECEIVE: {
      int pos_int, speed_int, torque_int;
      pos_int = (data[1] << 8) | data[2];
      speed_int = (data[3] << 4) | (data[4] >> 4);
      torque_int = (data[4] & 0xF) << 8 | data[5];

      motors[1]->pos_r = uint_to_float(pos_int, -12.5f, 12.5f, 16);
      motors[1]->angular_vel = uint_to_float(speed_int, -45.0f, 45.0f, 12);
      motors[1]->torque = uint_to_float(torque_int, -50.0f, 50.0f, 12);
    }
      break;

    case JOINT_RF_RECEIVE: {
      int pos_int, speed_int, torque_int;
      pos_int = (data[1] << 8) | data[2];
      speed_int = (data[3] << 4) | (data[4] >> 4);
      torque_int = (data[4] & 0xF) << 8 | data[5];

      motors[2]->pos_r = uint_to_float(pos_int, -12.5f, 12.5f, 16);
      motors[2]->angular_vel = uint_to_float(speed_int, -45.0f, 45.0f, 12);
      motors[2]->torque = uint_to_float(torque_int, -50.0f, 50.0f, 12);
    }
      break;

    case JOINT_RB_RECEIVE: {
      int pos_int, speed_int, torque_int;
      pos_int = (data[1] << 8) | data[2];
      speed_int = (data[3] << 4) | (data[4] >> 4);
      torque_int = (data[4] & 0xF) << 8 | data[5];

      motors[3]->pos_r = uint_to_float(pos_int, -12.5f, 12.5f, 16);
      motors[3]->angular_vel = uint_to_float(speed_int, -45.0f, 45.0f, 12);
      motors[3]->torque = uint_to_float(torque_int, -50.0f, 50.0f, 12);
    }
      break;

    default:break;
  }
}