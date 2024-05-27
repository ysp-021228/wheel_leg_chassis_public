#include "key_board.h"

key_board_t KeyBoard;
extern struct RCCtrl rc_ctrl;

//根据遥控器更改键鼠的按键状态
static void key_update(Key*key,uint16_t key_status,uint16_t cnt){
  if(key_status)
  {
    key->press_cnt++;
    if (key->status==KEY_RELAX)
    {
      key->last_status=KEY_RELAX;
      key->status=KEY_DOWN;
    } else if(key->status==KEY_DOWN&&(HAL_GetTick()-key->click_cnt)>325)
    {
      key->click_cnt=HAL_GetTick();
      key->last_status=KEY_DOWN;
      key->status=KEY_CLICK;
      if(key->click_flag==1)
      {
        key->click_flag=0;
      }
      else if(key->click_flag==0)
      {
        key->click_flag=1;
      }
    }

    if(key->press_cnt>cnt)
    {
      key->last_status=key->status;
      key->status=KEY_PRESS;
    }

  }
  else
  {
    key->last_status=key->status;
    key->status=KEY_RELAX;
    key->press_cnt=0;
  }

}

void update_pc_info(){

    //根据遥控器更改键鼠的按键状态
    /*键盘部分*/
    key_update(&KeyBoard.W,rc_ctrl.key.v&KEY_W,DEFAULT_CNT);//DEFAULT_CNT 为按键检测为长按的默认时间
    key_update(&KeyBoard.A,rc_ctrl.key.v&KEY_A,DEFAULT_CNT);
    key_update(&KeyBoard.S,rc_ctrl.key.v&KEY_S,DEFAULT_CNT);
    key_update(&KeyBoard.D,rc_ctrl.key.v&KEY_D,DEFAULT_CNT);
    key_update(&KeyBoard.Q,rc_ctrl.key.v&KEY_Q,DEFAULT_CNT);
    key_update(&KeyBoard.E,rc_ctrl.key.v&KEY_E,DEFAULT_CNT);
    key_update(&KeyBoard.R,rc_ctrl.key.v&KEY_R,DEFAULT_CNT);
    key_update(&KeyBoard.F,rc_ctrl.key.v&KEY_F,DEFAULT_CNT);
    key_update(&KeyBoard.G,rc_ctrl.key.v&KEY_G,DEFAULT_CNT);
    key_update(&KeyBoard.CTRL,rc_ctrl.key.v&KEY_CTRL,DEFAULT_CNT);
    key_update(&KeyBoard.SHIFT,rc_ctrl.key.v&KEY_SHIFT,DEFAULT_CNT);
    key_update(&KeyBoard.B,rc_ctrl.key.v&KEY_B,DEFAULT_CNT);

/*
    key_update(&KeyBoard.Z,rc_ctrl.key.v&KEY_Z);
    key_update(&KeyBoard.X,rc_ctrl.key.v&KEY_X);
    key_update(&KeyBoard.C,rc_ctrl.key.v&KEY_C);
    key_update(&KeyBoard.V,rc_ctrl.key.v&KEY_V);
    */

    /*鼠标部分*/
    key_update(&KeyBoard.Mouse_l,rc_ctrl.mouse.press_l&MOUSE_YES,MOUSE_CLICK_L_CNT);
    key_update(&KeyBoard.Mouse_r,rc_ctrl.mouse.press_r&MOUSE_YES,MOUSE_CLICK_R_CNT);

}


