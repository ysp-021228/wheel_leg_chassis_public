#include "vmc.h"
#include "user_lib.h"
#include "math.h"
#include "chassis.h"
#include "joint.h"
#include "filter.h"

static float phi_0_error;
static float steer_compensatory_torque;
struct MovingAverageFilter Fn_filter_L, Fn_filter_R;
static void Matrix_multiply(int rows1, int cols1, float matrix1[rows1][cols1],
                            int rows2, int cols2, float matrix2[rows2][cols2],
                            float result[rows1][cols2]) {
  if (cols1 != rows2)
    return;

  // Perform matrix multiplication
  for (int i = 0; i < rows1; ++i) {
    for (int j = 0; j < cols2; ++j) {
      result[i][j] = 0;
      for (int k = 0; k < cols1; ++k) {
        result[i][j] += matrix1[i][k] * matrix2[k][j];
      }
    }
  }
}

static void vmc_phi_update(struct Leg *leg_L, struct Leg *leg_R, const struct ChassisPhysicalConfig *physical_config) {
  leg_L->vmc.forward_kinematics.fk_phi.phi1 =
      PI - (-get_joint_motors()[1].pos_r - physical_config->mechanical_leg_limit_angle);
  leg_L->vmc.forward_kinematics.fk_phi.phi4 =
      (get_joint_motors() + 0)->pos_r - physical_config->mechanical_leg_limit_angle;

  leg_R->vmc.forward_kinematics.fk_phi.phi1 =
      PI - ((get_joint_motors() + 2)->pos_r - physical_config->mechanical_leg_limit_angle);
  leg_R->vmc.forward_kinematics.fk_phi.phi4 =
      -(get_joint_motors() + 3)->pos_r - physical_config->mechanical_leg_limit_angle;

}

static void forward_kinematics(struct Leg *leg_L,
                               struct Leg *leg_R,
                               const struct ChassisPhysicalConfig *physical_config) {
  //LEG_L
  leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x =
      cosf(leg_L->vmc.forward_kinematics.fk_phi.phi1) * physical_config->l1;
  leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y =
      sinf(leg_L->vmc.forward_kinematics.fk_phi.phi1) * physical_config->l1;
  leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x =
      cosf(leg_L->vmc.forward_kinematics.fk_phi.phi4) * physical_config->l4
          + physical_config->l5;
  leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y =
      sinf(leg_L->vmc.forward_kinematics.fk_phi.phi4) * physical_config->l5;

  float L_A0 = (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x
      - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x) * 2.f * physical_config->l2;
  float L_B0 = (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y
      - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y) * 2.f * physical_config->l2;
  float L_BD_sq = (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x
      - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x)
      * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x
          - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_x)
      + (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y
          - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y)
          * (leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y
              - leg_L->vmc.forward_kinematics.fk_point_coordinates.b_y);
  float L_C0 = physical_config->l2 * physical_config->l2 + L_BD_sq
      - physical_config->l3 * physical_config->l3;

  float temp = L_A0 * L_A0 + L_B0 * L_B0 - L_C0 * L_C0;
  float y = L_B0 + sqrtf(ABS(temp));
  float x = L_A0 + L_C0;
  leg_L->vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

  leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x =
      physical_config->l1 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi1)
          + physical_config->l2 * cosf(leg_L->vmc.forward_kinematics.fk_phi.phi2);
  leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y =
      physical_config->l1 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi1)
          + physical_config->l2 * sinf(leg_L->vmc.forward_kinematics.fk_phi.phi2);
  y = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y
      - leg_L->vmc.forward_kinematics.fk_point_coordinates.d_y;
  x = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x
      - leg_L->vmc.forward_kinematics.fk_point_coordinates.d_x;
  leg_L->vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

  temp = (leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
      * (leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
      + leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y
          * leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y;
  leg_L->vmc.forward_kinematics.fk_L0.L0_last = leg_L->vmc.forward_kinematics.fk_L0.L0;
  leg_L->vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));
  leg_L->vmc.forward_kinematics.fk_L0.L0_dot_last = leg_L->vmc.forward_kinematics.fk_L0.L0_dot;
  leg_L->vmc.forward_kinematics.fk_L0.L0_dot =
      (leg_L->vmc.forward_kinematics.fk_L0.L0 - leg_L->vmc.forward_kinematics.fk_L0.L0_last)
          / (CHASSIS_PERIOD * 0.001f);
  leg_L->vmc.forward_kinematics.fk_L0.L0_ddot =
      (leg_L->vmc.forward_kinematics.fk_L0.L0_dot - leg_L->vmc.forward_kinematics.fk_L0.L0_dot_last)
          / (CHASSIS_PERIOD * 0.001f);
  y = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_y;
  x = leg_L->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f;
  leg_L->vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);

  /***LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R LEG_R***/

  leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x =
      cosf(leg_R->vmc.forward_kinematics.fk_phi.phi1) * physical_config->l1;
  leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y =
      sinf(leg_R->vmc.forward_kinematics.fk_phi.phi1) * physical_config->l1;
  leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x =
      cosf(leg_R->vmc.forward_kinematics.fk_phi.phi4) * physical_config->l4
          + physical_config->l5;
  leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y =
      sinf(leg_R->vmc.forward_kinematics.fk_phi.phi4) * physical_config->l4;

  float R_A0 = (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x
      - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x) * 2.f * physical_config->l2;
  float R_B0 = (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y
      - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y) * 2.f * physical_config->l2;
  float R_BD_sq = (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x
      - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x)
      * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x
          - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_x)
      + (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y
          - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y)
          * (leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y
              - leg_R->vmc.forward_kinematics.fk_point_coordinates.b_y);
  float R_C0 = physical_config->l2 * physical_config->l2 + R_BD_sq - physical_config->l3 * physical_config->l3;

  temp = R_A0 * R_A0 + R_B0 * R_B0 - R_C0 * R_C0;
  y = R_B0 + sqrtf(ABS(temp));
  x = R_A0 + R_C0;
  leg_R->vmc.forward_kinematics.fk_phi.phi2 = 2.0f * atan2f(y, x);

  leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x =
      physical_config->l1 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi1)
          + physical_config->l2 * cosf(leg_R->vmc.forward_kinematics.fk_phi.phi2);
  leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y =
      physical_config->l1 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi1)
          + physical_config->l2 * sinf(leg_R->vmc.forward_kinematics.fk_phi.phi2);
  y = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y
      - leg_R->vmc.forward_kinematics.fk_point_coordinates.d_y;
  x = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x
      - leg_R->vmc.forward_kinematics.fk_point_coordinates.d_x;
  leg_R->vmc.forward_kinematics.fk_phi.phi3 = atan2f(y, x);

  temp = (leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
      * (leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f)
      + leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y
          * leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y;
  leg_R->vmc.forward_kinematics.fk_L0.L0_last = leg_R->vmc.forward_kinematics.fk_L0.L0;
  leg_R->vmc.forward_kinematics.fk_L0.L0 = sqrtf(ABS(temp));
  leg_R->vmc.forward_kinematics.fk_L0.L0_dot_last = leg_R->vmc.forward_kinematics.fk_L0.L0_dot;
  leg_R->vmc.forward_kinematics.fk_L0.L0_dot =
      (leg_R->vmc.forward_kinematics.fk_L0.L0 - leg_R->vmc.forward_kinematics.fk_L0.L0_last)
          / (CHASSIS_PERIOD * 0.001f);
  leg_R->vmc.forward_kinematics.fk_L0.L0_ddot =
      (leg_R->vmc.forward_kinematics.fk_L0.L0_dot - leg_R->vmc.forward_kinematics.fk_L0.L0_dot_last)
          / (CHASSIS_PERIOD * 0.001f);
  y = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_y;
  x = leg_R->vmc.forward_kinematics.fk_point_coordinates.c_x - physical_config->l5 * 0.5f;
  leg_R->vmc.forward_kinematics.fk_phi.phi0 = atan2f(y, x);

  phi_0_error = leg_L->vmc.forward_kinematics.fk_phi.phi0 - leg_R->vmc.forward_kinematics.fk_phi.phi0;
}

static void forward_dynamics(struct VMC *vmc, const struct ChassisPhysicalConfig *physical_config) {
  if (vmc == NULL) { return; }

  vmc->J_F_to_T.E.x1_1 =
      physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
          * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
          / (vmc->forward_kinematics.fk_L0.L0
              * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

  vmc->J_F_to_T.E.x1_2 =
      physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
          * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
          / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

  vmc->J_F_to_T.E.x2_1 =
      physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
          * cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
          / (vmc->forward_kinematics.fk_L0.L0
              * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2));

  vmc->J_F_to_T.E.x2_2 =
      physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4)
          * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
          / sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi2);

  Matrix_multiply(2, 2, vmc->J_F_to_T.array, 2, 1, vmc->Fxy_set_point.array, vmc->T1_T4_set_point.array);
}

static void wheel_motors_torque_set(struct Chassis *chassis) {
  if (chassis == NULL) {
    return;
  }
  if (chassis->chassis_ctrl_mode != CHASSIS_SPIN) {
#if REMOTE
    chassis->wheel_turn_torque = pid_loop_calc(&chassis->chassis_vw_current_pid,
                                               chassis->imu_reference.yaw_angle,
                                               chassis->chassis_ctrl_info.yaw_angle_rad,
                                               PI,
                                               -PI);
#else
    float turn_speed = pid_calc(&chassis->chassis_vw_speed_pid, chassis->chassis_ctrl_info.yaw_angle_rad, 0);

    chassis->wheel_turn_torque = pid_calc(&chassis->chassis_spin_pid,
                                           chassis->imu_reference.yaw_gyro,
                                           turn_speed);

//    chassis->wheel_turn_torque = -pid_calc(&chassis->chassis_vw_current_pid,
//                                               chassis->chassis_ctrl_info.yaw_angle_rad,
//                                               0);

//    chassis->wheel_turn_torque = pid_loop_calc(&chassis->chassis_vw_current_pid,
//                                               chassis->imu_reference.yaw_angle,
//                                               chassis->chassis_ctrl_info.yaw_angle_rad,
//                                               PI,
//                                               -PI);
#endif
  } else {
//    chassis->wheel_turn_torque = pid_calc(&chassis->chassis_spin_pid,
//                                          chassis->imu_reference.yaw_gyro,
//                                          chassis->chassis_ctrl_info.spin_speed);
  }

  if (chassis->is_chassis_offground) {
    chassis->wheel_turn_torque = 0;
  }
  chassis->leg_L.wheel_torque = 0;
  chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.theta;//
  chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.theta_dot;//
  chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.x;
  chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.x_dot;
  chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.phi;//
  chassis->leg_L.wheel_torque += chassis->leg_L.state_variable_wheel_out.phi_dot;
  chassis->leg_L.wheel_torque += chassis->wheel_turn_torque;

  chassis->leg_R.wheel_torque = 0;
  chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.theta;
  chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.theta_dot;
  chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.x;
  chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.x_dot;
  chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.phi;
  chassis->leg_R.wheel_torque += chassis->leg_R.state_variable_wheel_out.phi_dot;
  chassis->leg_R.wheel_torque -= chassis->wheel_turn_torque;
  chassis->leg_R.wheel_torque *= -1;

  VAL_LIMIT(chassis->leg_L.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
  VAL_LIMIT(chassis->leg_R.wheel_torque, MIN_WHEEL_TORQUE, MAX_WHEEL_TORQUE);
}

static void joint_motors_torque_set(struct Chassis *chassis,
                                    const struct ChassisPhysicalConfig *chassis_physical_config) {
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point = 0;
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point = 0;

  steer_compensatory_torque = pid_calc(&chassis->chassis_leg_coordination_pid, phi_0_error, 0);

//R
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta;
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.theta_dot;
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.x;
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.x_dot;
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.phi;
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_R.state_variable_joint_out.phi_dot;
  chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point += steer_compensatory_torque;

//L
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta;
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.theta_dot;
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.x;
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.x_dot;
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.phi;
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point += chassis->leg_L.state_variable_joint_out.phi_dot;
  chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point -= steer_compensatory_torque;

  float L0_delta = pid_calc(&get_chassis()->chassis_roll_pid,
                            get_chassis()->imu_reference.roll_angle,
                            get_chassis()->chassis_ctrl_info.roll_angle_rad);

  chassis->leg_L.L0_set_point = chassis->chassis_ctrl_info.height_m + L0_delta;
  chassis->leg_R.L0_set_point = chassis->chassis_ctrl_info.height_m - L0_delta;

//  if(chassis->is_chassis_offground){
//    chassis->leg_L.L0_set_point = chassis->chassis_ctrl_info.height_m ;
//    chassis->leg_R.L0_set_point = chassis->chassis_ctrl_info.height_m ;
//  }

  VAL_LIMIT(chassis->leg_L.L0_set_point, 0.13f, 0.40f);
  VAL_LIMIT(chassis->leg_R.L0_set_point, 0.13f, 0.40f);

  pid_calc(&chassis->leg_L.ground_pid,
           chassis->leg_L.vmc.forward_kinematics.fk_L0.L0 * cosf(chassis->leg_L.state_variable_feedback.theta),
           chassis->leg_L.L0_set_point);
//  pid_calc(&chassis->leg_L.ground_pid,
//           chassis->leg_L.vmc.forward_kinematics.fk_L0.L0,
//           chassis->leg_L.L0_set_point);
  chassis->leg_L.vmc.Fxy_set_point.E.Fy_set_point =
      chassis->leg_L.ground_pid.out + chassis_physical_config->body_weight * GRAVITY_A * 0.5;

  pid_calc(&chassis->leg_R.ground_pid,
           chassis->leg_R.vmc.forward_kinematics.fk_L0.L0 * cosf(chassis->leg_R.state_variable_feedback.theta),
           chassis->leg_R.L0_set_point);
//  pid_calc(&chassis->leg_R.ground_pid,
//           chassis->leg_R.vmc.forward_kinematics.fk_L0.L0,
//           chassis->leg_R.L0_set_point);
  chassis->leg_R.vmc.Fxy_set_point.E.Fy_set_point =
      chassis->leg_R.ground_pid.out + chassis_physical_config->body_weight * GRAVITY_A * 0.5;

  forward_dynamics(&chassis->leg_L.vmc, chassis_physical_config);
  forward_dynamics(&chassis->leg_R.vmc, chassis_physical_config);

  chassis->leg_L.joint_virtual_torque = chassis->leg_L.vmc.Fxy_set_point.E.Tp_set_point;
  chassis->leg_L.joint_F_torque = chassis->leg_L.vmc.T1_T4_set_point.E.T1_set_point;//F
  chassis->leg_L.joint_B_torque = chassis->leg_L.vmc.T1_T4_set_point.E.T4_set_point;//B

  chassis->leg_R.joint_virtual_torque = chassis->leg_R.vmc.Fxy_set_point.E.Tp_set_point;
  chassis->leg_R.joint_F_torque = chassis->leg_R.vmc.T1_T4_set_point.E.T1_set_point;//F
  chassis->leg_R.joint_B_torque = chassis->leg_R.vmc.T1_T4_set_point.E.T4_set_point;//B

  VAL_LIMIT(chassis->leg_R.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
  VAL_LIMIT(chassis->leg_R.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
  VAL_LIMIT(chassis->leg_L.joint_F_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
  VAL_LIMIT(chassis->leg_L.joint_B_torque, MIN_JOINT_TORQUE, MAX_JOINT_TORQUE);
}

static void Inverse_Kinematics(struct VMC *vmc,
                               float w1,
                               float w4,
                               const struct ChassisPhysicalConfig *chassis_physical_config) {
  if (vmc == NULL) {
    return;
  }
  vmc->W_fdb.E.w1_fdb = w1;
  vmc->W_fdb.E.w4_fdb = w4;

  vmc->J_w_to_v.E.x1_1 = (chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0)
      * sinf(vmc->forward_kinematics.fk_phi.phi3)
      * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
      - chassis_physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0)
          * sinf(vmc->forward_kinematics.fk_phi.phi2)
          * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4))
      / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);
  vmc->J_w_to_v.E.x1_2 = (chassis_physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0)
      * sinf(vmc->forward_kinematics.fk_phi.phi3)
      * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
      + chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi0)
          * sinf(vmc->forward_kinematics.fk_phi.phi2)
          * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4))
      / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);
  vmc->J_w_to_v.E.x2_1 = (-chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi0)
      * cosf(vmc->forward_kinematics.fk_phi.phi3)
      * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2)
      + chassis_physical_config->l4 * cosf(vmc->forward_kinematics.fk_phi.phi0)
          * cosf(vmc->forward_kinematics.fk_phi.phi2)
          * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4))
      / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);
  vmc->J_w_to_v.E.x2_2 = -(chassis_physical_config->l1 * cosf(vmc->forward_kinematics.fk_phi.phi0)
      * cosf(vmc->forward_kinematics.fk_phi.phi3)
      * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2) * chassis_physical_config->l4
      * sinf(vmc->forward_kinematics.fk_phi.phi0) * cosf(vmc->forward_kinematics.fk_phi.phi2)
      * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4))
      / sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi3);

  Matrix_multiply(2, 2, vmc->J_w_to_v.array, 2, 1, vmc->W_fdb.array, vmc->V_fdb.array);
  vmc->V_fdb.E.w0_fdb /= vmc->forward_kinematics.fk_L0.L0;
}

static void Inverse_Dynamics(struct VMC *vmc,
                             float T1,
                             float T4,
                             const struct ChassisPhysicalConfig *chassis_physical_config) {
  if (vmc == NULL) {
    return;
  }
  vmc->T1_T4_fdb.E.T1_fdb = T1;
  vmc->T1_T4_fdb.E.T4_fdb = T4;

  vmc->J_T_to_F.E.x1_1 =
      vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
          / (chassis_physical_config->l1
              * sinf(vmc->forward_kinematics.fk_phi.phi1 - vmc->forward_kinematics.fk_phi.phi2));
  vmc->J_T_to_F.E.x1_2 =
      vmc->forward_kinematics.fk_L0.L0 * sinf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
          / (chassis_physical_config->l4
              * sinf(vmc->forward_kinematics.fk_phi.phi4 - vmc->forward_kinematics.fk_phi.phi3));
  vmc->J_T_to_F.E.x2_1 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi2)
      / (chassis_physical_config->l1 * sinf(vmc->forward_kinematics.fk_phi.phi2 - vmc->forward_kinematics.fk_phi.phi1));
  vmc->J_T_to_F.E.x2_2 = cosf(vmc->forward_kinematics.fk_phi.phi0 - vmc->forward_kinematics.fk_phi.phi3)
      / (chassis_physical_config->l4 * sinf(vmc->forward_kinematics.fk_phi.phi3 - vmc->forward_kinematics.fk_phi.phi4));

  Matrix_multiply(2, 2, vmc->J_T_to_F.array, 2, 1, vmc->T1_T4_fdb.array, vmc->Fxy_fdb.array);
}

static void fn_cal(struct Leg *leg, float az, const struct ChassisPhysicalConfig *chassis_physical_config) {
  if (leg == NULL) {
    return;
  }
  float P = leg->vmc.Fxy_fdb.E.Tp_fdb * sinf(leg->state_variable_feedback.theta) / leg->vmc.forward_kinematics.fk_L0.L0
      + leg->vmc.Fxy_fdb.E.Fy_fdb * cosf(leg->state_variable_feedback.theta);

  float leg_az;
  leg_az = az - leg->vmc.forward_kinematics.fk_L0.L0_ddot * cosf(leg->state_variable_feedback.theta)
      + 2 * leg->vmc.forward_kinematics.fk_L0.L0_dot * leg->state_variable_feedback.theta_dot
          * sinf(leg->state_variable_feedback.theta)
      + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_feedback.theta_ddot
          * sinf(leg->state_variable_feedback.theta)
      + leg->vmc.forward_kinematics.fk_L0.L0 * leg->state_variable_feedback.theta_dot
          * leg->state_variable_feedback.theta_dot * cosf(leg->state_variable_feedback.theta);

  float Fn_raw = P + chassis_physical_config->wheel_weight * 9.8f + chassis_physical_config->wheel_weight * leg_az;
//  float Fn_raw = P;

  if (leg->leg_index == L) {
    updateFilter(&Fn_filter_L, Fn_raw);
    leg->Fn = getFilteredValue(&Fn_filter_L);
  } else if (leg->leg_index == R) {
    updateFilter(&Fn_filter_R, Fn_raw);
    leg->Fn = getFilteredValue(&Fn_filter_R);
  }
}
/*******************************************************************************
 *                                     VMC                                     *
 *******************************************************************************/
void vmc_ctrl(struct Chassis *chassis, const struct ChassisPhysicalConfig *chassis_physical_config) {
  vmc_phi_update(&chassis->leg_L, &chassis->leg_R, chassis_physical_config);
  forward_kinematics(&chassis->leg_L, &chassis->leg_R, chassis_physical_config);
  wheel_motors_torque_set(chassis);
  joint_motors_torque_set(chassis, chassis_physical_config);
  Inverse_Kinematics(&chassis->leg_L.vmc,
                     -(get_joint_motors() + 1)->angular_vel,
                     get_joint_motors()->angular_vel,
                     chassis_physical_config);
  Inverse_Kinematics(&chassis->leg_R.vmc,
                     (get_joint_motors() + 2)->angular_vel,
                     -(get_joint_motors() + 3)->angular_vel,
                     chassis_physical_config);
  Inverse_Dynamics(&chassis->leg_L.vmc,
                   -(get_joint_motors() + 1)->torque,
                   get_joint_motors()->torque,
                   chassis_physical_config);
  Inverse_Dynamics(&chassis->leg_R.vmc,
                   (get_joint_motors() + 2)->torque,
                   -(get_joint_motors() + 3)->torque,
                   chassis_physical_config);
  fn_cal(&chassis->leg_L, chassis->imu_reference.robot_az, chassis_physical_config);
  fn_cal(&chassis->leg_R, chassis->imu_reference.robot_az, chassis_physical_config);
}