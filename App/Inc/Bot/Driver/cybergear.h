#ifndef CYBERGEAR_H
#define CYBERGEAR_H

#include "main.h"
#include "string.h"
#include "can_device.h"

/*******************************************************************************
 *                                  Motor ID                                   *
 *******************************************************************************/
#define LF_MOTOR_ID (0x66)
#define LB_MOTOR_ID (0x67)
#define RB_MOTOR_ID (0x68)
#define RF_MOTOR_ID (0x69)

#define MI_MOTOR_MASTER_ID 0x10
#define MI_MOTOR_SINGLE_NUM 256

#define MI_MOTOR_P_MIN (-12.5f)
#define MI_MOTOR_P_MAX 12.5f
#define MI_MOTOR_V_MIN (-30.0f)
#define MI_MOTOR_V_MAX 30.0f
#define MI_MOTOR_KP_MIN 0.0f
#define MI_MOTOR_KP_MAX 500.0f
#define MI_MOTOR_KD_MIN 0.0f
#define MI_MOTOR_KD_MAX 5.0f
#define MI_MOTOR_T_MIN (-12.0f)
#define MI_MOTOR_T_MAX 12.0f

#define MI_MOTOR_RUN_MODE_INDEX 0x7005          //0:运控模式 1:位置模式 2:速度模式 3:电流模式
#define MI_MOTOR_IQ_REF_INDEX 0x7006            //电流模式Iq指令 -27~27A
#define MI_MOTOR_SPD_REF_INDEX 0x700A           //转速模式转速指令 -30~30rad/s
#define MI_MOTOR_LIMIT_TORQUE_INDEX 0x700B      //力矩限制 0~12N.m
#define MI_MOTOR_CUR_KP_INDEX 0x7010            //电流的 Kp default=0.125
#define MI_MOTOR_CUR_KI_INDEX 0x7011            //电流的 Ki default=0.0158
#define MI_MOTOR_CUR_FILTER_GAIN_INDEX 0x7014   //电流滤波系数 0~1.0 default=0.1
#define MI_MOTOR_LOC_REF_INDEX 0x7016           //位置模式位置指令 -12.5~12.5rad
#define MI_MOTOR_LIMIT_SPD_INDEX 0x7017         //速度限制 0~30rad/s
#define MI_MOTOR_LIMIT_CUR_INDEX 0x7018         //电流限制 0~27A

/**
 * @brief 电机数据结构体
 */
struct CyberGearData {
  CAN_HandleTypeDef canHandleTypeDef;
  uint8_t id;
  uint16_t data;
  uint8_t mode;
  uint8_t tx_data[8];
  float pos_r;
  float angular_vel;
  float torque;
  float temp;
  uint8_t fault[4];
  uint8_t warning[4];
  uint8_t index[2];
  uint8_t ref[4];
  float  current;
  uint32_t last_heartbeat_timestamp_ms;
};

extern struct CyberGearData cybergears_1[MI_MOTOR_SINGLE_NUM];
extern struct CyberGearData cybergears_2[MI_MOTOR_SINGLE_NUM];

extern void cyber_gear_can_send(CAN_HandleTypeDef canHandleTypeDef, struct CyberGearData *mi_motor_data);

extern void mi_motor_id_get(struct CyberGearData *mi_motor_data);

extern void
cyber_gear_control_mode(struct CyberGearData *mi_motor_data, float torque, float MechPosition, float speed, float kp,
                        float kd);

extern void cyber_gear_init(CAN_HandleTypeDef canHandleTypeDef, uint8_t id, struct CyberGearData *mi_motor_data);

extern void cyber_gear_enable(struct CyberGearData *mi_motor_data);

extern void mi_motor_disable(struct CyberGearData *mi_motor_data);

extern void mi_motor_clear_err(struct CyberGearData *mi_motor_data);

extern void mi_motor_set_zero(struct CyberGearData *mi_motor_data);

extern void mi_motor_set_id(struct CyberGearData *mi_motor_data, uint8_t set_id);

extern void mi_motor_read(struct CyberGearData *mi_motor_data, uint16_t index);

extern void cyber_gear_mode(struct CyberGearData *mi_motor_data, uint8_t mode);

extern void mi_motor_write(struct CyberGearData *mi_motor_data, uint16_t index, float ref);

extern void mi_motor_data_get(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef rx_header, const uint8_t rx_data[8]);


#endif //CYBERGEAR_H
