#include "joint.h"
#include "cmsis_os.h"

struct Dm8009 joint[4];

void joint_init() {
  dm8009_init(&joint[LF], JOINT_LF_SEND);
  dm8009_init(&joint[LB], JOINT_LB_SEND);
  dm8009_init(&joint[RF], JOINT_RF_SEND);
  dm8009_init(&joint[RB], JOINT_RB_SEND);
  set_dm8009_enable(CAN_2, JOINT_LF_SEND);
  set_dm8009_enable(CAN_2, JOINT_LB_SEND);
  osDelay(2);
  set_dm8009_enable(CAN_2, JOINT_RF_SEND);
  set_dm8009_enable(CAN_2, JOINT_RB_SEND);
}

void set_joint_torque(float joint_LF_torque, float joint_LB_torque, float joint_RF_torque, float joint_RB_torque) {
  set_dm8009_torque(CAN_2, JOINT_LF_SEND, joint_LF_torque);
  osDelay(1);
  set_dm8009_torque(CAN_2, JOINT_LB_SEND, joint_LB_torque);
  osDelay(1);
  set_dm8009_torque(CAN_2, JOINT_RF_SEND, joint_RF_torque);
  osDelay(1);
  set_dm8009_torque(CAN_2, JOINT_RB_SEND, joint_RB_torque);
}

struct Dm8009* get_joint_motors(){
  return joint;
}
