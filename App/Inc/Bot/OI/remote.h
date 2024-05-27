#ifndef DEMO1_REMOTE_H
#define DEMO1_REMOTE_H

#include "stdint-gcc.h"
#include "main.h"
#include "stdint-gcc.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
#define RC_s_R 0
#define RC_s_L 1
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_W            ((uint16_t)1 << 0)
#define KEY_S            ((uint16_t)1 << 1)
#define KEY_A            ((uint16_t)1 << 2)
#define KEY_D            ((uint16_t)1 << 3)
#define KEY_SHIFT        ((uint16_t)1 << 4)
#define KEY_CTRL         ((uint16_t)1 << 5)
#define KEY_Q            ((uint16_t)1 << 6)
#define KEY_E            ((uint16_t)1 << 7)
#define KEY_R            ((uint16_t)1 << 8)
#define KEY_F            ((uint16_t)1 << 9)
#define KEY_G            ((uint16_t)1 << 10)
#define KEY_Z            ((uint16_t)1 << 11)
#define KEY_X            ((uint16_t)1 << 12)
#define KEY_C            ((uint16_t)1 << 13)
#define KEY_V            ((uint16_t)1 << 14)
#define KEY_B            ((uint16_t)1 << 15)
/* ----------------------- Mouse Key Definition-------------------------------- */
#define MOUSE_YES 1
#define MOUSE_NO 0
/* ----------------------- Data Struct ------------------------------------- */
struct RCCtrl {
  __packed struct {
    int16_t ch[5];
    char s[2];
  } rc;
  __packed struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
  } mouse;
  __packed struct {
    uint16_t v;
  } key;

  uint32_t last_heartbeat_timestamp_ms;

}__packed;

/* ----------------------- Internal Data ----------------------------------- */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void RC_unable(void);
void RC_restart(uint16_t dma_buf_num);
void remote_control_init(void);
struct RCCtrl * get_rc_ctrl();
void check_is_rc_online(struct RCCtrl *rc_ctrl);

#endif
