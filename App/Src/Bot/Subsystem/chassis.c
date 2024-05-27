#include <stdbool.h>
#include <math.h>
#include "chassis.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "Atti.h"
#include "remote.h"
#include "user_lib.h"
#include "dm_8009.h"
#include "error.h"
#include "filter.h"
#include "joint.h"
#include "gimbal_communication.h"

#define DEBUG 0

const struct ChassisPhysicalConfig chassis_physical_config = {0.075f,
                                                              13.250f,
                                                              0.32f,
                                                              0.2618f,
                                                              0.15f,
                                                              0.27f,
                                                              0.27f,
                                                              0.15f,
                                                              0.15f};


/*******************************************************************************
 *                                Remote control                               *
 *******************************************************************************/
#define CHASSIS_X_CHANNEL 1
#define CHASSIS_Z_CHANNEL 2
#define CHASSIS_PIT_CHANNEL 3
#define CHASSIS_ROLL_CHANNEL 0

#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_YAW_INCREMENT (MAX_CHASSIS_YAW_INCREMENT/660)
#define RC_TO_PITCH ((MAX_PITCH-MIN_PITCH)/660)
#define RC_TO_ROLL ((MAX_ROLL-MIN_ROLL)/660)
#define RC_TO_L0  ((MAX_L0-MIN_L0)/1320)

/*******************************************************************************
 *                                PID parameters                               *
 *******************************************************************************/
#define CHASSIS_LEG_L0_PID_P 700
#define CHASSIS_LEG_L0_PID_I 0.02
#define CHASSIS_LEG_L0_PID_D 6000
#define CHASSIS_LEG_L0_PID_IOUT_LIMIT 10
#define CHASSIS_LEG_L0_PID_OUT_LIMIT 100000
//#define CHASSIS_LEG_L0_PID_OUT_LIMIT 0

#define CHASSIS_OFFGROUND_LEG_LO_PID_P 250
#define CHASSIS_OFFGROUND_LEG_L0_PID_I 0
#define CHASSIS_OFFGROUND_LEG_L0_PID_D 00000
#define CHASSIS_OFFGROUND_LEG_L0_PID_IOUT_LIMIT 3
#define CHASSIS_OFFGROUND_LEG_L0_PID_OUT_LIMIT 10000

#define CHASSIS_VW_CURRENT_PID_P (-5)
#define CHASSIS_VW_CURRENT_PID_I (-0.1)
#define CHASSIS_VW_CURRENT_PID_D (-800)
#define CHASSIS_VW_CURRENT_PID_IOUT_LIMIT 0.1
#define CHASSIS_VW_CURRENT_PID_OUT_LIMIT 10

//#define CHASSIS_VW_CURRENT_PID_P (-8.0)
//#define CHASSIS_VW_CURRENT_PID_I (-0.01)
//#define CHASSIS_VW_CURRENT_PID_D (-800)
//#define CHASSIS_VW_CURRENT_PID_IOUT_LIMIT 0.1
//#define CHASSIS_VW_CURRENT_PID_OUT_LIMIT 10

//#define CHASSIS_VW_CURRENT_PID_P 0.0
//#define CHASSIS_VW_CURRENT_PID_I 0
//#define CHASSIS_VW_CURRENT_PID_D -0
//#define CHASSIS_VW_CURRENT_PID_IOUT_LIMIT 0.2
//#define CHASSIS_VW_CURRENT_PID_OUT_LIMIT 0.5

#define CHASSIS_VW_SPEED_PID_P (-4.5)
#define CHASSIS_VW_SPEED_PID_I (-0.1)
#define CHASSIS_VW_SPEED_PID_D (20)
#define CHASSIS_VW_SPEED_PID_IOUT_LIMIT 0.1
#define CHASSIS_VW_SPEED_PID_OUT_LIMIT 3

#define CHASSIS_SPIN_PID_P (-5.5)
#define CHASSIS_SPIN_PID_I (-0.0)
#define CHASSIS_SPIN_PID_D (-5)
#define CHASSIS_SPIN_PID_IOUT_LIMIT 0.1
#define CHASSIS_SPIN_PID_OUT_LIMIT 10

#define CHASSIS_ROLL_PID_P 0.7
#define CHASSIS_ROLL_PID_I 0.005
#define CHASSIS_ROLL_PID_D 0.1
#define CHASSIS_ROLL_PID_IOUT_LIMIT 0.01
#define CHASSIS_ROLL_PID_OUT_LIMIT 0.3
//#define CHASSIS_ROLL_PID_OUT_LIMIT 0

#define CHASSIS_LEG_COORDINATION_PID_P 100
#define CHASSIS_LEG_COORDINATION_PID_I 0.0
#define CHASSIS_LEG_COORDINATION_PID_D 300
#define CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT 2
#define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 100
//#define CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT 0

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

static uint8_t rc_sw_R_last;
static bool init_flag = false;
static bool is_joint_enable = false;
struct Chassis chassis;
struct MovingAverageFilter robot_az_filter;
/*******************************************************************************
 *                                    Init                                     *
 *******************************************************************************/
static void chassis_pid_init() {
  pid_init(&chassis.chassis_vw_speed_pid,
           CHASSIS_VW_SPEED_PID_OUT_LIMIT,
           CHASSIS_VW_SPEED_PID_IOUT_LIMIT,
           CHASSIS_VW_SPEED_PID_P,
           CHASSIS_VW_SPEED_PID_I,
           CHASSIS_VW_SPEED_PID_D);

  pid_init(&chassis.chassis_vw_current_pid,
           CHASSIS_VW_CURRENT_PID_OUT_LIMIT,
           CHASSIS_VW_CURRENT_PID_IOUT_LIMIT,
           CHASSIS_VW_CURRENT_PID_P,
           CHASSIS_VW_CURRENT_PID_I,
           CHASSIS_VW_CURRENT_PID_D);

  pid_init(&chassis.chassis_spin_pid,
           CHASSIS_SPIN_PID_OUT_LIMIT,
           CHASSIS_SPIN_PID_IOUT_LIMIT,
           CHASSIS_SPIN_PID_P,
           CHASSIS_SPIN_PID_I,
           CHASSIS_SPIN_PID_D);

  pid_init(&chassis.leg_L.ground_pid,
           CHASSIS_LEG_L0_PID_OUT_LIMIT,
           CHASSIS_LEG_L0_PID_IOUT_LIMIT,
           CHASSIS_LEG_L0_PID_P,
           CHASSIS_LEG_L0_PID_I,
           CHASSIS_LEG_L0_PID_D);

  pid_init(&chassis.leg_R.ground_pid,
           CHASSIS_LEG_L0_PID_OUT_LIMIT,
           CHASSIS_LEG_L0_PID_IOUT_LIMIT,
           CHASSIS_LEG_L0_PID_P,
           CHASSIS_LEG_L0_PID_I,
           CHASSIS_LEG_L0_PID_D);

  pid_init(&chassis.chassis_roll_pid,
           CHASSIS_ROLL_PID_OUT_LIMIT,
           CHASSIS_ROLL_PID_IOUT_LIMIT,
           CHASSIS_ROLL_PID_P,
           CHASSIS_ROLL_PID_I,
           CHASSIS_ROLL_PID_D);

  pid_init(&chassis.chassis_leg_coordination_pid,
           CHASSIS_LEG_COORDINATION_PID_OUT_LIMIT,
           CHASSIS_LEG_COORDINATION_PID_IOUT_LIMIT,
           CHASSIS_LEG_COORDINATION_PID_P,
           CHASSIS_LEG_COORDINATION_PID_I,
           CHASSIS_LEG_COORDINATION_PID_D);
}

static void chassis_kalman_init(struct KalmanFilter *kalman_filter) {
  float dt = 0.005f;//单位s，后续传入的初始值都是ms为单位

  float R_init[4] = {0.0025f, 0,
                     0, 1.0f};//观测误差
  float Q_init[4] = {dt * dt, dt,
                     dt, 1,};//process noise
  float H_init[4] = {1, 0,
                     0, 1};
  float Xk_init[2] = {0, 0};

  float F_init[4] = {1, dt,
                     0, 1};
//  float restrict_init[3] = {0.03f, 0.005f, 0.1f};
  kalman_filter->H_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->H_data, 0, sizeof(float));
  memcpy(kalman_filter->H_data, H_init, sizeof(H_init));

  kalman_filter->F_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->F_data, 0, sizeof(float));
  memcpy(kalman_filter->F_data, F_init, sizeof(F_init));

  kalman_filter->R_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->R_data, 0, sizeof(float));
  memcpy(kalman_filter->R_data, R_init, sizeof(R_init));

  kalman_filter->Q_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->Q_data, 0, sizeof(float));
  memcpy(kalman_filter->Q_data, Q_init, sizeof(Q_init));

  kalman_filter->Xk_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->Xk_data, 0, sizeof(float));
  memcpy(kalman_filter->Xk_data, Xk_init, sizeof(Xk_init));

  kalman_filter->sigma_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->sigma_data, 0, sizeof(float));
  memcpy(kalman_filter->sigma_data, Q_init, sizeof(Q_init));

  kalman_filter->sigma_minus_data = (float *) malloc(sizeof(float) * 9);
  memset(kalman_filter->sigma_minus_data, 0, sizeof(float));
  memcpy(kalman_filter->sigma_minus_data, Q_init, sizeof(Q_init));

  kalman_Filter_Init(kalman_filter, 2, 0, 1, 2);
}

void chassis_init() {
  osDelay(2000);
  chassis.leg_L.leg_index = L;
  chassis.leg_R.leg_index = R;
  wheel_init();
  osDelay(2);
  joint_init();
  chassis_pid_init();
  chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
  chassis.is_chassis_offground = false;
  init_filter(&robot_az_filter);
  init_filter(&theta_ddot_filter_L);
  init_filter(&theta_ddot_filter_R);
  chassis.chassis_ctrl_info.spin_speed = 5.0f;
  vTaskSuspendAll();
  chassis_kalman_init(&chassis.leg_L.vx_kalman);
  chassis_kalman_init(&chassis.leg_R.vx_kalman);
  xTaskResumeAll();
}
/*******************************************************************************
 *                                Getter&Setter                                *
 *******************************************************************************/
static void get_IMU_info() {
  chassis.imu_reference.pitch_angle = -*(get_ins_angle() + 2);
  chassis.imu_reference.yaw_angle = -*(get_ins_angle() + 0);
  chassis.imu_reference.roll_angle = -*(get_ins_angle() + 1);

  chassis.imu_reference.pitch_gyro = -*(get_ins_gyro() + 0);
  chassis.imu_reference.yaw_gyro = -*(get_ins_gyro() + 2);
  chassis.imu_reference.roll_gyro = -*(get_ins_gyro() + 1);

  chassis.imu_reference.ax = -*(get_ins_accel() + 1);
  chassis.imu_reference.ay = *(get_ins_accel() + 0);
  chassis.imu_reference.az = *(get_ins_accel() + 2);

  chassis.imu_reference.ax_filtered = chassis.imu_reference.ax - GRAVITY_A * sinf(chassis.imu_reference.pitch_angle);
  chassis.imu_reference.ay_filtered = chassis.imu_reference.ay
      - GRAVITY_A * cosf(chassis.imu_reference.pitch_angle) *
          sinf(chassis.imu_reference.roll_angle);
  chassis.imu_reference.az_filtered = chassis.imu_reference.az
      - GRAVITY_A * cosf(chassis.imu_reference.pitch_angle) *
          cosf(chassis.imu_reference.roll_angle);
  float robot_az_raw = chassis.imu_reference.ax_filtered * sinf(chassis.imu_reference.pitch_angle)
      + chassis.imu_reference.ay_filtered * sinf(-chassis.imu_reference.roll_angle)
          * cosf(chassis.imu_reference.pitch_angle)
      + chassis.imu_reference.az_filtered * cosf(chassis.imu_reference.pitch_angle)
          * cosf(chassis.imu_reference.roll_angle);

  updateFilter(&robot_az_filter, robot_az_raw);
  chassis.imu_reference.robot_az = getFilteredValue(&robot_az_filter);
}

static bool is_chassis_ballanced() {
  if (ABS(chassis.imu_reference.pitch_angle) <= 0.04 && ABS(chassis.imu_reference.pitch_gyro) <= 0.2) {
    return true;
  } else {
    return false;
  }
}

static bool is_joints_reduced() {
  if (ABS(get_joint_motors()[0].pos_r) <= 0.174533 &&
      ABS(get_joint_motors()[1].pos_r) <= 0.174533 &&
      ABS(get_joint_motors()[2].pos_r) <= 0.174533 &&
      ABS(get_joint_motors()[3].pos_r) <= 0.174533) {
    return true;
  } else {
    return false;
  }
}

bool is_chassis_off_ground() {
  if ((chassis.is_chassis_offground == false) && (chassis.leg_L.Fn < 15 && chassis.leg_R.Fn < 15)) {
    chassis.is_chassis_offground = true;
    return true;
  } else if ((chassis.is_chassis_offground == true) && (chassis.leg_L.Fn > 25 || chassis.leg_R.Fn > 25)
      && (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 < 20 && chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 < 20)) {
    chassis.is_chassis_offground = false;
    return false;
  }
}

static void set_chassis_mode() {
  if (switch_is_down(get_rc_ctrl()->rc.s[RC_s_R])) {
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
    chassis.jump_state == NOT_READY;
  } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && init_flag == false) {
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_INIT;
  } else if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_R]) && init_flag == true) {
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
    chassis.jump_state = NOT_READY;
  } else if (switch_is_mid(rc_sw_R_last) && init_flag == true && switch_is_up(get_rc_ctrl()->rc.s[RC_s_R])) {
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_JUMP;
    chassis.jump_state = READY;
  }
  rc_sw_R_last = get_rc_ctrl()->rc.s[RC_s_R];
}

static void set_chassis_mode_from_gimbal_msg() {
  if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_DISABLE) {
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
  } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_ENABLE && init_flag == false) {
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_INIT;
  } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_ENABLE && init_flag == true) {
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_ENABLE;
  } else if (get_gimbal_msg()->chassis_ctrl_mode == CHASSIS_SIDEWAYS) {
    chassis.chassis_ctrl_mode_last = chassis.chassis_ctrl_mode;
    chassis.chassis_ctrl_mode = CHASSIS_SIDEWAYS;
  }
}

static void chassis_device_offline_handle() {
  check_is_rc_online(get_rc_ctrl());
  if (get_errors() != 0) {
    chassis.chassis_ctrl_mode = CHASSIS_DISABLE;
  }
}

static void set_chassis_ctrl_info() {
  chassis.chassis_ctrl_info.v_m_per_s = (float) (get_rc_ctrl()->rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX;

  chassis.chassis_ctrl_info.yaw_angle_rad -= (float) (get_rc_ctrl()->rc.ch[CHASSIS_Z_CHANNEL]) * -RC_TO_YAW_INCREMENT;
  if (chassis.chassis_ctrl_info.yaw_angle_rad >= PI) {
    chassis.chassis_ctrl_info.yaw_angle_rad -= 2 * PI;
  } else if (chassis.chassis_ctrl_info.yaw_angle_rad <= -PI) {
    chassis.chassis_ctrl_info.yaw_angle_rad += 2 * PI;
  }

  chassis.chassis_ctrl_info.pitch_angle_rad = (float) (get_rc_ctrl()->rc.ch[CHASSIS_PIT_CHANNEL]) * RC_TO_PITCH;

  chassis.chassis_ctrl_info.height_m = -(float) (get_rc_ctrl()->rc.ch[4]) * RC_TO_L0 + DEFAULT_L0;
  VAL_LIMIT(chassis.chassis_ctrl_info.height_m, MIN_L0, MAX_L0)
}

static void set_chassis_ctrl_info_from_gimbal_msg() {
  chassis.chassis_ctrl_info.v_m_per_s = get_gimbal_msg()->chassis_ctrl_info.v_m_per_s;
  chassis.chassis_ctrl_info.yaw_angle_rad = get_gimbal_msg()->chassis_ctrl_info.yaw_angle_rad;
  chassis.chassis_ctrl_info.roll_angle_rad = get_gimbal_msg()->chassis_ctrl_info.roll_angle_rad;
  chassis.chassis_ctrl_info.height_m = get_gimbal_msg()->chassis_ctrl_info.height_m;
}

static void chassis_motor_cmd_send() {
#if DEBUG
  set_joint_torque(0, 0, 0, 0);
  osDelay(2);
  set_wheel_torque(0, 0);

#else
  set_joint_torque(-chassis.leg_L.joint_F_torque,
                   -chassis.leg_L.joint_B_torque,
                   chassis.leg_R.joint_F_torque,
                   chassis.leg_R.joint_B_torque);

//  set_joint_torque(0, 0, 0, 0);

  set_wheel_torque(-chassis.leg_L.wheel_torque, -chassis.leg_R.wheel_torque);

//  set_wheel_torque(0, 0);

#endif
}

static void chassis_vx_kalman_run() {
  kalman_Filter_Predict(&chassis.leg_L.vx_kalman);
  kalman_Filter_Predict(&chassis.leg_R.vx_kalman);

  chassis.leg_L.kalman_measure[0] = (float) get_wheel_motors()[0].angular_vel * chassis_physical_config.wheel_radius;
  chassis.leg_L.kalman_measure[1] = chassis.imu_reference.ax_filtered;

  chassis.leg_R.kalman_measure[0] = (float) -get_wheel_motors()[1].angular_vel * chassis_physical_config.wheel_radius;
  chassis.leg_R.kalman_measure[1] = chassis.imu_reference.ax_filtered;

  chassis.leg_L.kalman_result = kalman_Filter_Update(&chassis.leg_L.vx_kalman, chassis.leg_L.kalman_measure);
  chassis.leg_R.kalman_result = kalman_Filter_Update(&chassis.leg_R.vx_kalman, chassis.leg_R.kalman_measure);
}

struct Chassis *get_chassis() {
  return &chassis;
}
/*******************************************************************************
 *                                    Task                                     *
 *******************************************************************************/
static void chassis_init_task() {
  chassis.chassis_ctrl_info.yaw_angle_rad = chassis.imu_reference.yaw_angle;
  if (!is_joint_enable) {
    set_dm8009_enable(CAN_2, JOINT_LF_SEND);
    set_dm8009_enable(CAN_2, JOINT_LB_SEND);
    HAL_Delay(2);
    set_dm8009_enable(CAN_2, JOINT_RF_SEND);
    set_dm8009_enable(CAN_2, JOINT_RB_SEND);
    HAL_Delay(2);
    is_joint_enable = true;
    return;
  }
  if (is_joints_reduced()) {
    init_flag = true;
  } else {
    set_dm8009_MIT(CAN_2, JOINT_LF_SEND, 0, 0, 20, 5, 0);
    set_dm8009_MIT(CAN_2, JOINT_LB_SEND, 0, 0, 20, 5, 0);
    HAL_Delay(2);
    set_dm8009_MIT(CAN_2, JOINT_RF_SEND, 0, 0, 20, 5, 0);
    set_dm8009_MIT(CAN_2, JOINT_RB_SEND, 0, 0, 20, 5, 0);
  }
}

static void chassis_enable_task() {
  chassis.leg_L.ground_pid.p = CHASSIS_LEG_L0_PID_P;
  chassis.leg_R.ground_pid.p = CHASSIS_LEG_L0_PID_P;
  chassis.leg_L.ground_pid.d = CHASSIS_LEG_L0_PID_D;
  chassis.leg_R.ground_pid.d = CHASSIS_LEG_L0_PID_D;
//  is_chassis_off_ground();
  lqr_ctrl(&chassis);
  vmc_ctrl(&chassis, &chassis_physical_config);
  chassis_vx_kalman_run();
}

static void chassis_disable_task() {
  chassis.leg_L.wheel_torque = 0;
  chassis.leg_R.wheel_torque = 0;
  chassis.leg_L.joint_F_torque = 0;
  chassis.leg_L.joint_B_torque = 0;
  chassis.leg_R.joint_F_torque = 0;
  chassis.leg_R.joint_B_torque = 0;

  chassis.leg_L.state_variable_feedback.x = 0;
  chassis.leg_R.state_variable_feedback.x = 0;

  chassis.chassis_ctrl_info.yaw_angle_rad = chassis.imu_reference.yaw_angle;

  init_flag = false;
  is_joint_enable = false;
  lk9025_set_enable(CAN_2, WHEEL_L_SEND);
  lk9025_set_enable(CAN_2, WHEEL_R_SEND);
}

static void chassis_jump_task() {
  switch (chassis.jump_state) {
    case READY:chassis.chassis_ctrl_info.height_m = 0.13f;
      chassis.leg_L.ground_pid.p = 600;
      chassis.leg_R.ground_pid.p = 600;
      if ((ABS(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0_dot) <= 0.03f
          && ABS(chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 - chassis.leg_L.L0_set_point) <= 0.07)
          && (ABS(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0_dot) <= 0.03f
              && ABS(chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 - chassis.leg_R.L0_set_point) <= 0.07f)) {
        if (switch_is_mid(get_rc_ctrl()->rc.s[RC_s_L])) {
          chassis.jump_state = STRETCHING;
        }
      }
      break;

    case STRETCHING:chassis.chassis_ctrl_info.height_m = 0.35f;
      chassis.leg_L.ground_pid.p = 1500;
      chassis.leg_R.ground_pid.p = 1500;
      if (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 > 0.30
          && chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 > 0.30) {
        chassis.jump_state = SHRINKING;
      }
      break;

    case SHRINKING:chassis.chassis_ctrl_info.height_m = 0.15f;
      chassis.leg_L.ground_pid.p = 1000;
      chassis.leg_L.ground_pid.d = 1800;
      chassis.leg_R.ground_pid.p = 1000;
      chassis.leg_R.ground_pid.d = 1800;
      if (chassis.leg_L.vmc.forward_kinematics.fk_L0.L0 < 0.25
          && chassis.leg_R.vmc.forward_kinematics.fk_L0.L0 < 0.25) {
        chassis.jump_state = STRETCHING_AGAIN;
      }
      break;

    case STRETCHING_AGAIN:chassis.chassis_ctrl_info.height_m = 0.35f;
      chassis.leg_L.ground_pid.p = 600;
      chassis.leg_R.ground_pid.p = 600;
      if (chassis.imu_reference.robot_az <= -5) {
        chassis.jump_state = FALLING;
      }
      break;

    case FALLING:chassis.chassis_ctrl_info.height_m = DEFAULT_L0;
      chassis.leg_L.ground_pid.p = 600;
      chassis.leg_R.ground_pid.p = 600;
      if (chassis.leg_R.Fn >= 30 || chassis.leg_L.Fn >= 30) {
        chassis.jump_state = LANDING;
      }
      break;

    case LANDING:chassis.leg_L.ground_pid.p = 600;
      chassis.leg_R.ground_pid.p = 600;
      break;

    default:break;
  }
  lqr_ctrl(&chassis);
  vmc_ctrl(&chassis, &chassis_physical_config);
}

extern void chassis_task(void const *pvParameters) {

  chassis_init();

  TickType_t last_wake_time = xTaskGetTickCount();
  while (1) {

    vTaskSuspendAll();

    get_IMU_info();

#if REMOTE
    set_chassis_mode();

    set_chassis_ctrl_info();
#else
    set_chassis_mode_from_gimbal_msg();
    set_chassis_ctrl_info_from_gimbal_msg();
#endif
    chassis_device_offline_handle();

    switch (chassis.chassis_ctrl_mode) {
      case CHASSIS_INIT:
//        xTaskResumeAll();
        chassis_init_task();
//        chassis_enable_task();
//        is_chassis_off_ground();
        break;

      case CHASSIS_ENABLE:chassis_enable_task();
        break;

      case CHASSIS_DISABLE:chassis_disable_task();
        break;

      case CHASSIS_SPIN:chassis.chassis_ctrl_info.v_m_per_s = 0;
        chassis.chassis_ctrl_info.yaw_angle_rad = 0;
        chassis_enable_task();
        break;

      case CHASSIS_JUMP:chassis_jump_task();
        break;

      case CHASSIS_SIDEWAYS:chassis_enable_task();
        break;

      default:break;
    }

    xTaskResumeAll();
    chassis_motor_cmd_send();

    vTaskDelayUntil(&last_wake_time, CHASSIS_PERIOD);
  }
}
