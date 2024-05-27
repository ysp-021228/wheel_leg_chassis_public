#include "cybergear.h"
#include "can.h"

/*
 * С�׵��ʹ��˵��
 * 1.��Ҫ��CAN�жϻص���������mi_motor_data_get()��������ȷ�����ݽ���
 * 2.��ʹ��ǰʹ��mi_motor_init()����ʵ�ֵ����ʼ�� ������CAN���߾���͵��id����Ӧ�Ľṹ������
 * 3.ʹ��mi_motor_mode()�������õ��ģʽ 0:�˿�ģʽ 1:λ��ģʽ 2:�ٶ�ģʽ 3:����ģʽ��Ĭ��Ϊ�˿�ģʽ��
 * 4.ʹ��mi_motor_enable()����ʹ�ܵ��
 * 5.ʹ��mi_motor_disable()����ʧ�ܵ��
 * 6.ʹ��mi_motor_control_mode()�������õ���˿�ģʽ���˿�ģʽ��
 * 7.ʹ��mi_motor_write()�������õ��������λ��ģʽ���ٶ�ģʽ������ģʽ��
 * 8.ʹ��mi_motor_read()������ȡ������ò���
 */

/**
 * @brief С�׵��CAN1��������
 */
struct CyberGearData cybergears_1[MI_MOTOR_SINGLE_NUM];

/**
 * @brief С�׵��CAN2��������
 */
struct CyberGearData cybergears_2[MI_MOTOR_SINGLE_NUM];

/**
 * @brief С�׵��CAN ��չ֡ID��Ϣ������
 */
union {
  uint32_t ExtId;
  struct {
    uint32_t id: 8;
    uint32_t data: 16;
    uint32_t mode: 5;
    uint32_t res: 3;
  };
} exCanIdInfo;

static int float_to_uint(float x, float x_min, float x_max);

static float uint_to_float(int x_int, float x_min, float x_max);

static void mi_motor_can_send(struct CyberGearData *mi_motor_data);

/**
 * @brief С�׵��CAN����ת������
 * @param x
 * @param x_min
 * @param x_max
 * @return
 */
static int float_to_uint(float x, float x_min, float x_max) {
  float span = x_max - x_min;
  float offset = x_min;
  if (x > x_max) x = x_max;
  else if (x < x_min) x = x_min;
  return (int) ((x - offset) * ((float) ((1 << 16) - 1)) / span);
}

/**
 * @brief С�׵��CAN����ת������
 * @param x_int
 * @param x_min
 * @param x_max
 * @return
 */
static float uint_to_float(int x_int, float x_min, float x_max) {
  //converts unsigned int to float, given range
  float span = x_max - x_min;
  float offset = x_min;
  return ((float) x_int) * span / ((float) ((1 << 16) - 1)) + offset;
}

/**
 * @brief С�׵��CAN���ͺ���
 * @param mi_motor_data ������ݽṹ��
 */
static void mi_motor_can_send(struct CyberGearData *mi_motor_data) {
  CAN_TxHeaderTypeDef tx_header;
  uint8_t tx_data[8];
  uint32_t send_mail_box;
//  send_mail_box=get_free_can_mailbox();

  exCanIdInfo.mode = mi_motor_data->mode;
  exCanIdInfo.id = mi_motor_data->id;
  exCanIdInfo.data = mi_motor_data->data;
  exCanIdInfo.res = 0;

  tx_header.ExtId = exCanIdInfo.ExtId;
  tx_header.IDE = CAN_ID_EXT;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;

  memcpy(tx_data, mi_motor_data->tx_data, 8);

  HAL_CAN_AddTxMessage(&mi_motor_data->canHandleTypeDef, &tx_header, tx_data, &send_mail_box);
}

/**
 * @brief С�׵��ID��ȡ����
 * @param mi_motor_data ������ݽṹ��
 */
void mi_motor_id_get(struct CyberGearData *mi_motor_data) {
  mi_motor_data->mode = 0;
  union {
    uint16_t ex_data;
    struct {
      uint16_t master_id: 8;
      uint16_t res: 8;
    };
  } ex_data;
  ex_data.master_id = MI_MOTOR_MASTER_ID;
  ex_data.res = 0;
  mi_motor_data->data = ex_data.ex_data;
  memset(&mi_motor_data->tx_data, 0, 8);
  mi_motor_can_send(mi_motor_data);
}

/**
 * @brief ����˿�ģʽ����
 * @param mi_motor_data ������ݽṹ��
 * @param torque ����ָ�� -12~12N.m
 * @param MechPosition ����ű������Ƕ�ָ�� -12.5~12.5rad
 * @param speed ����ٶ�ָ�� -30~30rad/s
 * @param kp λ�û�kp
 * @param kd �ٶȻ�kd
 */
void cyber_gear_control_mode(struct CyberGearData *mi_motor_data, float torque, float MechPosition, float speed, float kp,
                             float kd) {
  mi_motor_data->mode = 1;
  mi_motor_data->data = float_to_uint(torque, MI_MOTOR_T_MIN, MI_MOTOR_T_MAX);

  mi_motor_data->tx_data[0] = float_to_uint(MechPosition, MI_MOTOR_P_MIN, MI_MOTOR_P_MAX) >> 8;
  mi_motor_data->tx_data[1] = float_to_uint(MechPosition, MI_MOTOR_P_MIN, MI_MOTOR_P_MAX);
  mi_motor_data->tx_data[2] = float_to_uint(speed, MI_MOTOR_V_MIN, MI_MOTOR_V_MAX) >> 8;
  mi_motor_data->tx_data[3] = float_to_uint(speed, MI_MOTOR_V_MIN, MI_MOTOR_V_MAX);
  mi_motor_data->tx_data[4] = float_to_uint(kp, MI_MOTOR_KP_MIN, MI_MOTOR_KP_MAX) >> 8;
  mi_motor_data->tx_data[5] = float_to_uint(kp, MI_MOTOR_KP_MIN, MI_MOTOR_KP_MAX);
  mi_motor_data->tx_data[6] = float_to_uint(kd, MI_MOTOR_KD_MIN, MI_MOTOR_KD_MAX) >> 8;
  mi_motor_data->tx_data[7] = float_to_uint(kd, MI_MOTOR_KD_MIN, MI_MOTOR_KD_MAX);
  mi_motor_can_send(mi_motor_data);
}

/**
 * @brief С�׵����ʼ���������ڵ��ʹ��ǰ������ã�
 * @param canHandleTypeDef ���CAN����ѡ�� hcan1/hcan2
 * @param id ���id
 * @param mi_motor_data ������ݽṹ��
 */
void cyber_gear_init(CAN_HandleTypeDef canHandleTypeDef, uint8_t id, struct CyberGearData *mi_motor_data) {
  union {
    uint16_t ex_data;
    struct {
      uint16_t master_id: 8;
      uint16_t res: 8;
    };
  } ex_data;
  ex_data.master_id = MI_MOTOR_MASTER_ID;
  ex_data.res = 0;
  mi_motor_data->data = ex_data.ex_data;
  mi_motor_data->id = id;
  mi_motor_data->canHandleTypeDef = canHandleTypeDef;
  memset(&mi_motor_data->tx_data, 0, 8);
}

/**
 * @brief С�׵��ʹ�ܺ���
 * @param mi_motor_data ������ݽṹ��
 */
void cyber_gear_enable(struct CyberGearData *mi_motor_data) {
  mi_motor_data->mode = 3;
  union {
    uint16_t ex_data;
    struct {
      uint16_t master_id: 8;
      uint16_t res: 8;
    };
  } ex_data;
  ex_data.master_id = MI_MOTOR_MASTER_ID;
  ex_data.res = 0;
  mi_motor_data->data = ex_data.ex_data;
  memset(&mi_motor_data->tx_data, 0, 8);
  mi_motor_can_send(mi_motor_data);
}

/**
 * @brief С�׵��ʧ�ܺ���
 * @param mi_motor_data ������ݽṹ��
 */
void mi_motor_disable(struct CyberGearData *mi_motor_data) {
  mi_motor_data->mode = 4;
  union {
    uint16_t ex_data;
    struct {
      uint16_t master_id: 8;
      uint16_t res: 8;
    };
  } ex_data;
  ex_data.master_id = MI_MOTOR_MASTER_ID;
  ex_data.res = 0;
  mi_motor_data->data = ex_data.ex_data;
  memset(&mi_motor_data->tx_data, 0, 8);
  mi_motor_can_send(mi_motor_data);
}

/**
 * @brief С�׵�����������
 * @param mi_motor_data ������ݽṹ��
 */
void mi_motor_clear_err(struct CyberGearData *mi_motor_data) {
  mi_motor_data->mode = 4;
  union {
    uint16_t ex_data;
    struct {
      uint16_t master_id: 8;
      uint16_t res: 8;
    };
  } ex_data;
  ex_data.master_id = MI_MOTOR_MASTER_ID;
  ex_data.res = 0;
  mi_motor_data->data = ex_data.ex_data;
  memset(&mi_motor_data->tx_data, 0, 8);
  mi_motor_data->tx_data[0] = 1;
  mi_motor_can_send(mi_motor_data);
}

/**
 * @brief С�׵��������㺯��
 * @param mi_motor_data ������ݽṹ��
 */
void mi_motor_set_zero(struct CyberGearData *mi_motor_data) {
  mi_motor_data->mode = 6;
  union {
    uint16_t ex_data;
    struct {
      uint16_t master_id: 8;
      uint16_t res: 8;
    };
  } ex_data;
  ex_data.master_id = MI_MOTOR_MASTER_ID;
  ex_data.res = 0;
  mi_motor_data->data = ex_data.ex_data;
  memset(&mi_motor_data->tx_data, 0, 8);
  mi_motor_data->tx_data[0] = 1;
  mi_motor_can_send(mi_motor_data);
}

/**
 * @brief С�׵������id����
 * @param mi_motor_data ������ݽṹ��
 * @param set_id �����µ�idֵ
 */
void mi_motor_set_id(struct CyberGearData *mi_motor_data, uint8_t set_id) {
  mi_motor_data->mode = 7;
  union {
    uint16_t ex_data;
    struct {
      uint16_t master_id: 8;
      uint16_t set_id: 8;
    };
  } ex_data;
  ex_data.master_id = MI_MOTOR_MASTER_ID;
  ex_data.set_id = set_id;
  mi_motor_data->data = ex_data.ex_data;
  memset(&mi_motor_data->tx_data, 0, 8);
  mi_motor_can_send(mi_motor_data);
}

/**
 * @brief С�׵�����ݶ�ȡ����
 * @param mi_motor_data ������ݽṹ��
 * @param index ����
 */
void mi_motor_read(struct CyberGearData *mi_motor_data, uint16_t index) {
  mi_motor_data->mode = 17;
  union {
    uint16_t ex_data;
    struct {
      uint16_t master_id: 8;
      uint16_t res: 8;
    };
  } ex_data;
  ex_data.master_id = MI_MOTOR_MASTER_ID;
  ex_data.res = 0;
  mi_motor_data->data = ex_data.ex_data;
  memset(&mi_motor_data->tx_data, 0, 8);
  memcpy(&mi_motor_data->tx_data[0], &index, 2);
  mi_motor_can_send(mi_motor_data);
}

/**
 * @brief С�׵��ģʽ���ú���
 * @param mi_motor_data ������ݽṹ��
 * @param mode ģʽ 0:�˿�ģʽ 1:λ��ģʽ 2:�ٶ�ģʽ 3:����ģʽ
 */
void cyber_gear_mode(struct CyberGearData *mi_motor_data, uint8_t mode) {
  uint16_t index = MI_MOTOR_RUN_MODE_INDEX;
  mi_motor_data->mode = 18;
  union {
    uint16_t ex_data;
    struct {
      uint16_t master_id: 8;
      uint16_t res: 8;
    };
  } ex_data;
  ex_data.master_id = MI_MOTOR_MASTER_ID;
  ex_data.res = 0;
  mi_motor_data->data = ex_data.ex_data;
  memset(&mi_motor_data->tx_data, 0, 8);
  memcpy(&mi_motor_data->tx_data[0], &index, 2);
  memcpy(&mi_motor_data->tx_data[4], &mode, 1);
  mi_motor_can_send(mi_motor_data);
}

/**
 * @brief С�׵������д�뺯��
 * @param mi_motor_data ������ݽṹ��
 * @param index ����
 * @param ref   ����ֵ
 */
void mi_motor_write(struct CyberGearData *mi_motor_data, uint16_t index, float ref) {
  mi_motor_data->mode = 18;
  union {
    uint16_t ex_data;
    struct {
      uint16_t master_id: 8;
      uint16_t res: 8;
    };
  } ex_data;
  ex_data.master_id = MI_MOTOR_MASTER_ID;
  ex_data.res = 0;
  mi_motor_data->data = ex_data.ex_data;
  memset(&mi_motor_data->tx_data, 0, 8);
  memcpy(&mi_motor_data->tx_data[0], &index, 2);
  memcpy(&mi_motor_data->tx_data[4], &ref, 4);
  mi_motor_can_send(mi_motor_data);
}

/**
 * @brief С�׵��CAN���ݶ�ȡ��������Ҫ��CAN�жϻص��е��ã�
 * @param hcan
 * @param rx_header
 * @param rx_data
 */
void mi_motor_data_get(CAN_HandleTypeDef *hcan, CAN_RxHeaderTypeDef rx_header, const uint8_t rx_data[8]) {
  exCanIdInfo.ExtId = rx_header.ExtId;
  union {
    uint16_t ex_data;
    struct {
      uint16_t motor_id: 8;
      uint16_t res: 8;
    };
  } ex_data;
  ex_data.ex_data = exCanIdInfo.data;
  if (hcan == &hcan1) {
    if (exCanIdInfo.mode == 2) {
      cybergears_1[ex_data.motor_id].id = ex_data.motor_id;
      cybergears_1[ex_data.motor_id].mode = exCanIdInfo.mode;
      cybergears_1[ex_data.motor_id].data = exCanIdInfo.data;

      uint16_t raw_angle = (int16_t) ((rx_data[0] << 8) | rx_data[1]);
      uint16_t raw_speed = (int16_t) ((rx_data[2] << 8) | rx_data[3]);
      uint16_t raw_torque = (int16_t) ((rx_data[4] << 8) | rx_data[5]);
      uint16_t raw_temp = (int16_t) ((rx_data[6] << 8) | rx_data[7]);

      cybergears_1[ex_data.motor_id].pos_r = uint_to_float(raw_angle, MI_MOTOR_P_MIN, MI_MOTOR_P_MAX);
      cybergears_1[ex_data.motor_id].angular_vel = uint_to_float(raw_speed, MI_MOTOR_V_MIN, MI_MOTOR_V_MAX);
      cybergears_1[ex_data.motor_id].torque = uint_to_float(raw_torque, MI_MOTOR_T_MIN, MI_MOTOR_T_MAX);
      cybergears_1[ex_data.motor_id].temp = (float) raw_temp / 10.0f;

    } else if (exCanIdInfo.mode == 17) {
      memcpy(cybergears_1[ex_data.motor_id].index, rx_data, 2);
      memcpy(cybergears_1[ex_data.motor_id].ref, &rx_data[4], 4);
    } else if (exCanIdInfo.mode == 21) {
      memcpy(cybergears_1[ex_data.motor_id].fault, rx_data, 4);
      memcpy(cybergears_1[ex_data.motor_id].warning, &rx_data[4], 4);
    }
  } else if (hcan == &hcan2) {
    if (exCanIdInfo.mode == 2) {
      cybergears_2[ex_data.motor_id].id = ex_data.motor_id;
      cybergears_2[ex_data.motor_id].mode = exCanIdInfo.mode;
      cybergears_2[ex_data.motor_id].data = exCanIdInfo.data;

      uint16_t raw_angle = (int16_t) ((rx_data[0] << 8) | rx_data[1]);
      uint16_t raw_speed = (int16_t) ((rx_data[2] << 8) | rx_data[3]);
      uint16_t raw_torque = (int16_t) ((rx_data[4] << 8) | rx_data[5]);
      uint16_t raw_temp = (int16_t) ((rx_data[6] << 8) | rx_data[7]);

      cybergears_2[ex_data.motor_id].pos_r = uint_to_float(raw_angle, MI_MOTOR_P_MIN, MI_MOTOR_P_MAX);
      cybergears_2[ex_data.motor_id].angular_vel = uint_to_float(raw_speed, MI_MOTOR_V_MIN, MI_MOTOR_V_MAX);
      cybergears_2[ex_data.motor_id].torque = uint_to_float(raw_torque, MI_MOTOR_T_MIN, MI_MOTOR_T_MAX);
      cybergears_2[ex_data.motor_id].temp = (float) raw_temp / 10.0f;

    } else if (exCanIdInfo.mode == 17) {
      memcpy(cybergears_2[ex_data.motor_id].index, rx_data, 2);
      memcpy(cybergears_2[ex_data.motor_id].ref, &rx_data[4], 4);
    } else if (exCanIdInfo.mode == 21) {
      memcpy(cybergears_2[ex_data.motor_id].fault, rx_data, 4);
      memcpy(cybergears_2[ex_data.motor_id].warning, &rx_data[4], 4);
    }
  }
}