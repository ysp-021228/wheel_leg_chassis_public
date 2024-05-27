#ifndef CHASSIS_H
#define CHASSIS_H

#include "stdbool.h"
#include "joint.h"
#include "wheel.h"
#include "vmc.h"
#include "lqr.h"
#include "pid.h"
#include "vx_kalman_filter.h"
#define REMOTE 0

#define CHASSIS_PERIOD 5

/*******************************************************************************
 *                                    Limit                                    *
 *******************************************************************************/
#define MAX_CHASSIS_VX_SPEED 1.8f
#define MAX_CHASSIS_VW_TORQUE 0.5f
#define MAX_CHASSIS_YAW_INCREMENT 0.02f
#define MIN_L0 0.13f
#define MAX_L0 0.40f
#define DEFAULT_L0 0.22f
#define MAX_PITCH 0.174533f
#define MIN_PITCH (-0.174533f)
#define MAX_ROLL 0.12f
#define MIN_ROLL (-0.12f)
#define MAX_WHEEL_TORQUE 10.f
#define MIN_WHEEL_TORQUE (-10.f)
#define MAX_JOINT_TORQUE 40.f
#define MIN_JOINT_TORQUE (-40.f)

#define WHEEL_THETA_LIMIT      1.5f
#define WHEEL_THETA_DOT_LIMIT  0.5f
#define WHEEL_X_LIMIT          0.f
#define WHEEL_X_DOT_LIMIT      1.0f
#define WHEEL_PHI_LIMIT        1.0f
#define WHEEL_PHI_DOT_LIMIT    0.5f

#define JOINT_THETA_LIMIT      15.5f
#define JOINT_THETA_DOT_LIMIT  20.5f
#define JOINT_X_LIMIT          0.f
#define JOINT_X_DOT_LIMIT      10.0f
#define JOINT_PHI_LIMIT        10.5f
#define JOINT_PHI_DOT_LIMIT    10.0f

struct ChassisPhysicalConfig {
  float wheel_radius;
  float body_weight;
  float wheel_weight;
  float mechanical_leg_limit_angle;

  float l1, l2, l3, l4, l5;
};

enum ChassisCtrlMode {
  CHASSIS_DISABLE = 1,
  CHASSIS_ENABLE,
  CHASSIS_INIT,
  CHASSIS_JUMP,
  CHASSIS_SPIN,
  CHASSIS_SIDEWAYS,
};

enum LegIndex {
  R = 1,
  L = 0,
};

enum JumpState {
  NOT_READY,
  READY,
  STRETCHING,
  SHRINKING,
  STRETCHING_AGAIN,
  FALLING,
  LANDING,
};

struct LegFlag {
  bool OFF_GROUND_FLAG;
  bool IMPACT_FLAG;
};

struct IMUReference {
  //Triaxial Angle
  float pitch_angle;
  float yaw_angle;
  float roll_angle;

  //Triaxial angular velocity
  float pitch_gyro;
  float yaw_gyro;
  float roll_gyro;

  //Triaxial acceleration
  float ax;
  float ay;
  float az;

  //The triaxial acceleration removes the gravitational acceleration
  float ax_filtered;
  float ay_filtered;
  float az_filtered;

  float robot_az;
};

struct ChassisCtrlInfo {
  float v_m_per_s;
  float pitch_angle_rad;
  float yaw_angle_rad;
  float roll_angle_rad;
  float height_m;
  float spin_speed;
};

struct StateVariable {
  float theta;
  float theta_last;
  float theta_dot;
  float theta_dot_last;
  float theta_ddot;
  float x;
  float x_dot;
  float x_dot_last;
  float x_ddot;
  float phi;
  float phi_dot;
};

struct Leg {
  enum LegIndex leg_index;
  struct LegFlag leg_flag;
  struct StateVariable state_variable_feedback;
  struct StateVariable state_variable_set_point;
  struct StateVariable state_variable_error;
  struct StateVariable state_variable_wheel_out;
  struct StateVariable state_variable_joint_out;
  struct VMC vmc;
  struct Pid ground_pid;
  struct Pid offground_pid;
  struct KalmanFilter vx_kalman;
  float kalman_measure[2];
  float *kalman_result;
  float L0_set_point;
  float wheel_torque;
  float joint_F_torque;
  float joint_B_torque;
  float joint_virtual_torque;
  float Fn;
};

struct Chassis {
  enum ChassisCtrlMode chassis_ctrl_mode;
  enum ChassisCtrlMode chassis_ctrl_mode_last;
  enum JumpState jump_state;
  struct ChassisCtrlInfo chassis_ctrl_info;
  struct IMUReference imu_reference;
  struct Leg leg_L;
  struct Leg leg_R;
  struct Pid chassis_vw_speed_pid;
  struct Pid chassis_vw_current_pid;
  struct Pid chassis_spin_pid;
  struct Pid chassis_roll_pid;
  struct Pid chassis_leg_coordination_pid;
  bool is_chassis_offground;
  float wheel_turn_torque;
};

void chassis_init();

struct ChassisFeedbackInfo get_chassis_feedback_info();

struct Chassis *get_chassis();

bool is_chassis_off_ground();

extern void chassis_task(void const *pvParameters);

#endif //CHASSIS_H
