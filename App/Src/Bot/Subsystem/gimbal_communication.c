#include "gimbal_communication.h"
#include "cmsis_os.h"
#include "can_device.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "chassis.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static CAN_TxHeaderTypeDef
    tx_msg = {0x00, 0, CAN_ID_STD, CAN_RTR_DATA, 0x08, DISABLE};
static CAN_RxHeaderTypeDef rx_msg;

static uint8_t rx_data[9];

static struct GimbalMsg gimbal_msg;

static void send_chassis_angle_vel(enum CanType can_type,
                                   enum GimbalSendID CMD_ID,
                                   float chassis_pitch_vel,
                                   float chassis_yaw_vel) {
  tx_msg.StdId = CMD_ID;
  tx_msg.IDE = CAN_ID_STD;
  tx_msg.RTR = CAN_RTR_DATA;
  tx_msg.DLC = 0x08;

  uint32_t int_pitch_vel = *((uint32_t *) &chassis_pitch_vel);
  uint32_t int_yaw_vel = *((uint32_t *) &chassis_yaw_vel);

  uint8_t tx_data[8] = {0};
  tx_data[0] = (uint8_t) (int_pitch_vel >> 24);
  tx_data[1] = (uint8_t) (int_pitch_vel >> 16);
  tx_data[2] = (uint8_t) (int_pitch_vel >> 8);
  tx_data[3] = (uint8_t) (int_pitch_vel);
  tx_data[4] = (uint8_t) (int_yaw_vel >> 24);
  tx_data[5] = (uint8_t) (int_yaw_vel >> 16);
  tx_data[6] = (uint8_t) (int_yaw_vel >> 8);
  tx_data[7] = (uint8_t) (int_yaw_vel);

  uint32_t can1_send_mail_box = get_can1_free_mailbox();
  uint32_t can2_send_mail_box = get_can2_free_mailbox();

  if (can_type == CAN_1) {
    HAL_CAN_AddTxMessage(&hcan1, &tx_msg, tx_data, &can1_send_mail_box);
  } else if (can_type == CAN_2) {
    HAL_CAN_AddTxMessage(&hcan2, &tx_msg, tx_data, &can2_send_mail_box);
  }
}

struct GimbalMsg *get_gimbal_msg() {
  return &gimbal_msg;
}

uint8_t test = 0;

void gimbal_msg_unpack(uint32_t id, uint8_t data[]) {
  switch (id) {
    case CHASSIS_MODE_HEIGHT_INFO: {
      if (data[0] == 0) {
        test++;
      }
      gimbal_msg.chassis_ctrl_mode = (enum ChassisCtrlMode) data[1];
      if(gimbal_msg.chassis_ctrl_mode==0){
        test++;
      }
      uint32_t int_height = 0;
      int_height |= ((uint32_t) data[4]) << 24;
      int_height |= ((uint32_t) data[5]) << 16;
      int_height |= ((uint32_t) data[6]) << 8;
      int_height |= data[7];
      gimbal_msg.chassis_ctrl_info.height_m = *(float *) &int_height;
      break;
    }

    case CHASSIS_SPEED_INFO: {
      uint32_t int_v_m_per_s = 0;
      uint32_t int_yaw_angle_rad = 0;
      int_v_m_per_s |= ((uint32_t) data[0]) << 24;
      int_v_m_per_s |= ((uint32_t) data[1]) << 16;
      int_v_m_per_s |= ((uint32_t) data[2]) << 8;
      int_v_m_per_s |= data[3];

      int_yaw_angle_rad |= ((uint32_t) data[4]) << 24;
      int_yaw_angle_rad |= ((uint32_t) data[5]) << 16;
      int_yaw_angle_rad |= ((uint32_t) data[6]) << 8;
      int_yaw_angle_rad |= data[7];

      gimbal_msg.chassis_ctrl_info.v_m_per_s = *(float *) &int_v_m_per_s;
      gimbal_msg.chassis_ctrl_info.yaw_angle_rad = *(float *) &int_yaw_angle_rad;
      break;
    }

    case CHASSIS_ANGLE_INFO: {
      uint32_t int_roll_angle_rad = 0;

      int_roll_angle_rad |= ((uint32_t) data[0]) << 24;
      int_roll_angle_rad |= ((uint32_t) data[1]) << 16;
      int_roll_angle_rad |= ((uint32_t) data[2]) << 8;
      int_roll_angle_rad |= data[3];

      gimbal_msg.chassis_ctrl_info.roll_angle_rad = *(float *) &int_roll_angle_rad;
      break;
    }
  }
}

extern void gimbal_task(void const *pvParameters) {

  TickType_t last_wake_time = xTaskGetTickCount();
  while (1) {
    send_chassis_angle_vel(CAN_1,
                           CHASSIS_ANGLE_VEL_INFO,
                           get_chassis()->imu_reference.pitch_gyro,
                           get_chassis()->imu_reference.yaw_gyro);;

    vTaskDelayUntil(&last_wake_time, 5);
  }
}