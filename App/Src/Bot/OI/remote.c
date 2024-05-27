#include "remote.h"
#include "main.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "error.h"

#define RC_CHANNAL_ERROR_VALUE 700
#define RC_HEARTBEAT_TIMEOUT_MS 200

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

struct RCCtrl rc_ctrl;

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

  //enable the DMA transfer for the receiver request
  //使能DMA串口接收
  SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

  //enalbe idle interrupt
  //使能空闲中断
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

  //disable DMA
  //失效DMA
  __HAL_DMA_DISABLE(&hdma_usart3_rx);
  while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
  }

  hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
  //memory buffer 1
  //内存缓冲区1
  hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
  //memory buffer 2
  //内存缓冲区2
  hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
  //data length
  //数据长度
  hdma_usart3_rx.Instance->NDTR = dma_buf_num;
  //enable double memory buffer
  //使能双缓冲区
  SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

  //enable DMA
  //使能DMA
  __HAL_DMA_ENABLE(&hdma_usart3_rx);

}

void RC_unable(void)
{
  __HAL_UART_DISABLE(&huart3);
}

void RC_restart(uint16_t dma_buf_num)
{
  __HAL_UART_DISABLE(&huart3);
  __HAL_DMA_DISABLE(&hdma_usart3_rx);

  hdma_usart3_rx.Instance->NDTR = dma_buf_num;

  __HAL_DMA_ENABLE(&hdma_usart3_rx);
  __HAL_UART_ENABLE(&huart3);

}

static int16_t RC_abs(int16_t value)
{
  if (value > 0)
  {
    return value;
  }
  else
  {
    return -value;
  }
}

//在main函数中调用此函数
void remote_control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}


uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return 0;

    error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

void slove_RC_lost(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

void slove_data_error(void)
{
    RC_restart(SBUS_RX_BUF_NUM);
}

static void sbus_to_rc(volatile const uint8_t *sbus_buf, struct RCCtrl *rc_ctrl)
{
  if (sbus_buf == NULL || rc_ctrl == NULL)
  {
    return;
  }

  rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
  rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
  rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
      (sbus_buf[4] << 10)) &0x07ff;
  rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3

  rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
  rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
  rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
  rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
  rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
  rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
  rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
  rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
  rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

  rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
  rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
  if(rc_ctrl->rc.ch[2]<=10&&rc_ctrl->rc.ch[2]>=-10)
  {
    rc_ctrl->rc.ch[2]=0;
  }
  rc_ctrl->last_heartbeat_timestamp_ms = xTaskGetTickCount();
//    detect_handle(DETECT_REMOTE);
}

void check_is_rc_online(struct RCCtrl *rc_ctrl){
  if (rc_ctrl->last_heartbeat_timestamp_ms != 0) {
    uint32_t c620_heartbeat_period_ms = xTaskGetTickCount() - rc_ctrl->last_heartbeat_timestamp_ms;

    if (c620_heartbeat_period_ms < RC_HEARTBEAT_TIMEOUT_MS) {
      set_error(ERROR_RC_OFFLINE, false);
    } else {
      set_error(ERROR_RC_OFFLINE, true);
    }
  } else {
    set_error(ERROR_RC_OFFLINE, true);
  }
}

//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}


struct RCCtrl * get_rc_ctrl(){
  return &rc_ctrl;
}

