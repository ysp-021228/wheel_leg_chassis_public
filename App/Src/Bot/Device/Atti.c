/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler pos_r. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             ��Ҫ����������bmi088��������ist8310�������̬���㣬�ó�ŷ���ǣ�
  *             �ṩͨ��bmi088��data ready �ж�����ⲿ�������������ݵȴ��ӳ�
  *             ͨ��DMA��SPI�����ԼCPUʱ��.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "Device/Atti.h"

#include "main.h"

#include "cmsis_os.h"

#include "BMI088driver.h"
#include "ist8310driver.h"
#include "pid.h"
#include "AHRS.h"

#include "Calibrate.h"
//#include "detect_task.h"

#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm����

#define BMI088_BOARD_INSTALL_SPIN_MATRIX    \
    {0.0f, 1.0f, 0.0f},                     \
    {-1.0f, 0.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


#define IST8310_BOARD_INSTALL_SPIN_MATRIX   \
    {1.0f, 0.0f, 0.0f},                     \
    {0.0f, 1.0f, 0.0f},                     \
    {0.0f, 0.0f, 1.0f}                      \


/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
/**
  * @brief          ��ת������,���ٶȼƺʹ�����,��������Ư,��Ϊ�豸�в�ͬ��װ��ʽ
  * @param[out]     gyro: ������Ư����ת
  * @param[out]     accel: ������Ư����ת
  * @param[out]     mag: ������Ư����ת
  * @param[in]      bmi088: �����Ǻͼ��ٶȼ�����
  * @param[in]      ist8310: ����������
  * @retval         none
  */
static void imu_cali_slove(float gyro[3],
                           float accel[3],
                           float mag[3],
                           bmi088_real_data_t *bmi088,
                           ist8310_real_data_t *ist8310);

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(float temp);
/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_cmd_spi_dma(void);

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

Eulr_t Eulr;
bmi088_real_data_t bmi088_real_data;
float gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float gyro_offset[3];
float gyro_cali_offset[3];

float accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float accel_offset[3];
float accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
float mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
float mag_offset[3];
float mag_cali_offset[3];

static uint8_t first_temperate;
static const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static struct Pid imu_temp_pid;

static const float timing_time = 0.001f;   //tast run time , unit s.�������е�ʱ�� ��λ s


static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static float INS_accel[3] = {0.0f, 0.0f, 0.0f};
static float INS_mag[3] = {0.0f, 0.0f, 0.0f};
static float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler pos_r, unit rad.ŷ���� ��λ rad



void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num)
{
  SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
  SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);

  __HAL_SPI_ENABLE(&hspi1);


  //disable DMA
  //失效DMA
  __HAL_DMA_DISABLE(&hdma_spi1_rx);

  while(hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
  }

  __HAL_DMA_CLEAR_FLAG(&hdma_spi1_rx, DMA_LISR_TCIF2);

  hdma_spi1_rx.Instance->PAR = (uint32_t) & (SPI1->DR);
  //memory buffer 1
  //内存缓冲区1
  hdma_spi1_rx.Instance->M0AR = (uint32_t)(rx_buf);
  //data length
  //数据长度
  __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, num);

  __HAL_DMA_ENABLE_IT(&hdma_spi1_rx, DMA_IT_TC);


  //disable DMA
  //失效DMA
  __HAL_DMA_DISABLE(&hdma_spi1_tx);

  while(hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
  }


  __HAL_DMA_CLEAR_FLAG(&hdma_spi1_tx, DMA_LISR_TCIF3);

  hdma_spi1_tx.Instance->PAR = (uint32_t) & (SPI1->DR);
  //memory buffer 1
  //内存缓冲区1
  hdma_spi1_tx.Instance->M0AR = (uint32_t)(tx_buf);
  //data length
  //数据长度
  __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, num);


}

void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr)
{
  __HAL_DMA_DISABLE(&hdma_spi1_rx);
  __HAL_DMA_DISABLE(&hdma_spi1_tx);


  while(hdma_spi1_rx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_spi1_rx);
  }
  while(hdma_spi1_tx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_spi1_tx);
  }

  __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));
  __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmarx));
  __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmarx));
  __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmarx));
  __HAL_DMA_CLEAR_FLAG (hspi1.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmarx));

  __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmatx));
  __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi1.hdmatx));
  __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmatx));
  __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi1.hdmatx));
  __HAL_DMA_CLEAR_FLAG (hspi1.hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmatx));


  hdma_spi1_rx.Instance->M0AR = rx_buf;
  hdma_spi1_tx.Instance->M0AR = tx_buf;

  __HAL_DMA_SET_COUNTER(&hdma_spi1_rx, ndtr);
  __HAL_DMA_SET_COUNTER(&hdma_spi1_tx, ndtr);

  __HAL_DMA_ENABLE(&hdma_spi1_rx);
  __HAL_DMA_ENABLE(&hdma_spi1_tx);
}


/**
  * @brief          imu task, init bmi088, ist8310, calculate the euler pos_r
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          imu����, ��ʼ�� bmi088, ist8310, ����ŷ����
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

void INS_task(void const *pvParameters) {
  //wait a time
  osDelay(INS_TASK_INIT_TIME);
  while (BMI088_init()) {
    osDelay(100);
  }
  while (ist8310_init()) {
    osDelay(100);
  }

  BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
  //rotate and zero drift
  imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

//    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
  pid_init(&imu_temp_pid,
           TEMPERATURE_PID_MAX_OUT,
           TEMPERATURE_PID_MAX_IOUT,
           TEMPERATURE_PID_KP,
           TEMPERATURE_PID_KI,
           TEMPERATURE_PID_KD);
  AHRS_init(INS_quat, INS_accel, INS_mag);

  accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
  accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
  accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
  //get the handle of task
  //��ȡ��ǰ�������������
  INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

  //set spi frequency
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }

  SPI1_DMA_init((uint32_t) gyro_dma_tx_buf, (uint32_t) gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

  imu_start_dma_flag = 1;

  while (1) {
    //wait spi DMA tansmit done
    //�ȴ�SPI DMA����
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS) {
    }

    if (gyro_update_flag & (1 << IMU_NOTIFY_SHFITS)) {
      gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
      BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
    }

    if (accel_update_flag & (1 << IMU_UPDATE_SHFITS)) {
      accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                             bmi088_real_data.accel,
                             &bmi088_real_data.time);

    }

    if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS)) {
      accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
      imu_temp_control(bmi088_real_data.temp);
    }

    //rotate and zero drift
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);


    //���ٶȼƵ�ͨ�˲�
    //accel low-pass filter
    accel_fliter_1[0] = accel_fliter_2[0];
    accel_fliter_2[0] = accel_fliter_3[0];

    accel_fliter_3[0] =
        accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

    accel_fliter_1[1] = accel_fliter_2[1];
    accel_fliter_2[1] = accel_fliter_3[1];

    accel_fliter_3[1] =
        accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

    accel_fliter_1[2] = accel_fliter_2[2];
    accel_fliter_2[2] = accel_fliter_3[2];

    accel_fliter_3[2] =
        accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

    AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);
    get_angle(INS_quat, &INS_angle[0], &INS_angle[1], &INS_angle[2]);


    //because no use ist8310 and save time, no use
    if (mag_update_flag &= 1 << IMU_DR_SHFITS) {
      mag_update_flag &= ~(1 << IMU_DR_SHFITS);
      mag_update_flag |= (1 << IMU_SPI_SHFITS);
//            ist8310_read_mag(ist8310_real_data.mag);
    }

  }
}




/**
  * @brief          rotate the gyro, accel and mag, and calculate the zero drift, because sensors have
  *                 different install derection.
  * @param[out]     gyro: after plus zero drift and rotate
  * @param[out]     accel: after plus zero drift and rotate
  * @param[out]     mag: after plus zero drift and rotate
  * @param[in]      bmi088: gyro and accel data
  * @param[in]      ist8310: mag data
  * @retval         none
  */
/**
  * @brief          ��ת������,���ٶȼƺʹ�����,��������Ư,��Ϊ�豸�в�ͬ��װ��ʽ
  * @param[out]     gyro: ������Ư����ת
  * @param[out]     accel: ������Ư����ת
  * @param[out]     mag: ������Ư����ת
  * @param[in]      bmi088: �����Ǻͼ��ٶȼ�����
  * @param[in]      ist8310: ����������
  * @retval         none
  */
static void imu_cali_slove(float gyro[3],
                           float accel[3],
                           float mag[3],
                           bmi088_real_data_t *bmi088,
                           ist8310_real_data_t *ist8310) {
  for (uint8_t i = 0; i < 3; i++) {
    gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1]
        + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
    accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1]
        + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
    mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1]
        + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
  }
}

/**
  * @brief          control the temperature of bmi088
  * @param[in]      temp: the temperature of bmi088
  * @retval         none
  */
/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_temp_control(float temp) {
  uint16_t tempPWM;
  static uint8_t temp_constant_time = 0;
  if (first_temperate) {
    pid_calc(&imu_temp_pid, temp, get_control_temperature());
    if (imu_temp_pid.out < 0.0f) {
      imu_temp_pid.out = 0.0f;
    }
    tempPWM = (uint16_t) imu_temp_pid.out;
    IMU_temp_PWM(tempPWM);
  } else {
    //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
    //in beginning, max power
    if (temp > get_control_temperature()) {
      temp_constant_time++;
      if (temp_constant_time > 200) {
        //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
        //
        first_temperate = 1;
        imu_temp_pid.iout = MPU6500_TEMP_PWM_MAX / 2.0f;
      }
    }

    IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
  }
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     gyro_offset:zero drift
  * @param[in]      gyro:gyro data
  * @param[out]     offset_time_count: +1 auto
  * @retval         none
  */
/**
  * @brief          ������������Ư
  * @param[out]     gyro_offset:������Ư
  * @param[in]      gyro:���ٶ�����
  * @param[out]     offset_time_count: �Զ���1
  * @retval         none
  */
void gyro_offset_calc(float gyro_offset[3], float gyro[3], uint16_t *offset_time_count) {
  if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL) {
    return;
  }

  gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
  gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
  gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
  (*offset_time_count)++;
}

/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc
  * @retval         none
  */
/**
  * @brief          У׼������
  * @param[out]     �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[out]     �����ǵ���Ư���ɼ������ǵľ�ֹ�������Ϊoffset
  * @param[out]     �����ǵ�ʱ�̣�ÿ����gyro_offset���û��1,
  * @retval         none
  */
void INS_cali_gyro(float cali_scale[3], float cali_offset[3], uint16_t *time_count) {
  if (*time_count == 0) {
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
  }
  gyro_offset_calc(gyro_offset, INS_gyro, time_count);

  cali_offset[0] = gyro_offset[0];
  cali_offset[1] = gyro_offset[1];
  cali_offset[2] = gyro_offset[2];
  cali_scale[0] = 1.0f;
  cali_scale[1] = 1.0f;
  cali_scale[2] = 1.0f;

}

/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift,
  * @retval         none
  */
/**
  * @brief          У׼���������ã�����flash���������ط�����У׼ֵ
  * @param[in]      �����ǵı������ӣ�1.0fΪĬ��ֵ�����޸�
  * @param[in]      �����ǵ���Ư
  * @retval         none
  */
void INS_set_cali_gyro(float cali_scale[3], float cali_offset[3]) {
  gyro_cali_offset[0] = cali_offset[0];
  gyro_cali_offset[1] = cali_offset[1];
  gyro_cali_offset[2] = cali_offset[2];
  gyro_offset[0] = gyro_cali_offset[0];
  gyro_offset[1] = gyro_cali_offset[1];
  gyro_offset[2] = gyro_cali_offset[2];
}

/**
  * @brief          get the quat
  * @param[in]      none
  * @retval         the point of INS_quat
  */
/**
  * @brief          ��ȡ��Ԫ��
  * @param[in]      none
  * @retval         INS_quat��ָ��
  */
const float *get_INS_quat_point(void) {
  return INS_quat;
}
/**
  * @brief          get the euler pos_r, 0:yaw_angle, 1:pitch_angle, 2:roll_angle unit rad
  * @param[in]      none
  * @retval         the point of INS_angle
  */
/**
  * @brief          ��ȡŷ����, 0:yaw_angle, 1:pitch_angle, 2:roll_angle ��λ rad
  * @param[in]      none
  * @retval         INS_angle��ָ��
  */
const float *get_INS_angle_point(void) {
  return INS_angle;
}

/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll_angle-axis,unit rad/s
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll_angle�� ��λ rad/s
  * @param[in]      none
  * @retval         INS_gyro��ָ��
  */
extern const float *get_gyro_data_point(void) {
  return INS_gyro;
}
/**
  * @brief          get aceel, 0:x-axis, 1:y-axis, 2:roll_angle-axis unit m/s2
  * @param[in]      none
  * @retval         the point of INS_accel
  */
/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll_angle�� ��λ m/s2
  * @param[in]      none
  * @retval         INS_accel��ָ��
  */
extern const float *get_accel_data_point(void) {
  return INS_accel;
}
/**
  * @brief          get mag, 0:x-axis, 1:y-axis, 2:roll_angle-axis unit ut
  * @param[in]      none
  * @retval         the point of INS_mag
  */
/**
  * @brief          ��ȡ���ٶ�,0:x��, 1:y��, 2:roll_angle�� ��λ ut
  * @param[in]      none
  * @retval         INS_mag��ָ��
  */
extern const float *get_mag_data_point(void) {
  return INS_mag;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == INT1_ACCEL_Pin) {
//        detect_hook(BOARD_ACCEL_TOE);
    accel_update_flag |= 1 << IMU_DR_SHFITS;
    accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
    if (imu_start_dma_flag) {
      imu_cmd_spi_dma();
    }
  } else if (GPIO_Pin == INT1_GYRO_Pin) {
//        detect_hook(BOARD_GYRO_TOE);
    gyro_update_flag |= 1 << IMU_DR_SHFITS;
    if (imu_start_dma_flag) {
      imu_cmd_spi_dma();
    }
  } else if (GPIO_Pin == DRDY_IST8310_Pin) {
//        detect_hook(BOARD_MAG_TOE);
    mag_update_flag |= 1 << IMU_DR_SHFITS;
  } else if (GPIO_Pin == GPIO_PIN_0) {

    //wake up the task
    //��������
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
      static BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

  }

}

/**
  * @brief          open the SPI DMA accord to the value of imu_update_flag
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_cmd_spi_dma(void) {
  UBaseType_t uxSavedInterruptStatus;
  uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

  //���������ǵ�DMA����
  if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN)
      && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
      && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))) {
    gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
    gyro_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
    SPI1_DMA_enable((uint32_t) gyro_dma_tx_buf, (uint32_t) gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    return;
  }
  //�������ٶȼƵ�DMA����
  if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN)
      && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
      && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))) {
    accel_update_flag &= ~(1 << IMU_DR_SHFITS);
    accel_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    SPI1_DMA_enable((uint32_t) accel_dma_tx_buf, (uint32_t) accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    return;
  }

  if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN)
      && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
      && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS))) {
    accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
    accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    SPI1_DMA_enable((uint32_t) accel_temp_dma_tx_buf, (uint32_t) accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    return;
  }
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void Atti_DMA2_Stream2_IRQHandler(void) {

  if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET) {
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

    //gyro read over
    //�����Ƕ�ȡ���
    if (gyro_update_flag & (1 << IMU_SPI_SHFITS)) {
      gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
      gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

      HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

    }

    //accel read over
    //���ٶȼƶ�ȡ���
    if (accel_update_flag & (1 << IMU_SPI_SHFITS)) {
      accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
      accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

      HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }
    //temperature read over
    //�¶ȶ�ȡ���
    if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS)) {
      accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
      accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

      HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }

    imu_cmd_spi_dma();

    if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS)) {
      gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
      __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
    }
  }
}

float *get_ins_angle() {
  return INS_angle;
}

float *get_ins_gyro() {
  return INS_gyro;
}

float *get_ins_accel() {
  return INS_accel;
}

