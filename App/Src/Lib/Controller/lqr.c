#include <stddef.h>
#include "lqr.h"
#include "chassis.h"
#include "user_lib.h"
#include "math.h"
#include "remote.h"
#include "buzzer.h"
#include "filter.h"

#define BALANCE_WHEEL_R 0.06f //Æ½ºâ±øÂÖ×Ó°ë¾¶m

struct MovingAverageFilter theta_ddot_filter_L, theta_ddot_filter_R;

float init_wheel_K_L[6] = {-17.162366, -2.016433, -1.130678, -1.530085, 15.895931, 2.388100};
float init_wheel_K_R[6] = {-17.162366, -2.016433, -1.130678, -1.530085, 15.895931, 2.388100};
float init_joint_K_L[6] = {15.857528, 2.386820, 1.485220, 1.953273, 37.371067, 4.026670};
float init_joint_K_R[6] = {15.857528, 2.386820, 1.485220, 1.953273, 37.371067, 4.026670};

float wheel_K_L[6] = {0, 0, 0, 0, 0, 0};
float joint_K_L[6] = {0, 0, 0, 0, 0, 0};

float wheel_K_R[6] = {0, 0, 0, 0, 0, 0};
float joint_K_R[6] = {0, 0, 0, 0, 0, 0};

float wheel_fitting_factor[6][4] = {
    {-3945.962686,1386.357115,-303.377566,-9.841736},
    {185.809596,-47.460473,-10.963130,-3.510211},

    {2149.555396,-548.398009,41.567644,-9.390456},
    {5897.550760,-1570.441736,148.107260,-12.032064},

    {-2689.197901,833.595073,-159.535590,34.867672},
    {-1623.531916,455.331016,-55.815181,6.615042}
};
float joint_fitting_factor[6][4] = {
    {8717.667565,-2655.443299,309.112880,5.719143},
    {214.372510,-54.238421,1.185273,3.680965},

    {4377.797631,-1123.925311,85.526107,4.215017},
    {523.300172,-41.945220,-23.068638,6.788821},

    {-772.319168,-51.170237,90.545619,30.141439},
    {7.458022,-56.865652,21.357116,2.244028}
};

/////////////////////////////////////////////////////////////////////////////////////////////////>>

void chassis_K_matrix_fitting(float L0, float K[6], const float KL[6][4]) {
  for (int i = 0; i < 6; i++) {
    K[i] = KL[i][0] * powf(L0, 3) + KL[i][1] * powf(L0, 2) + KL[i][2] * powf(L0, 1) + KL[i][3] * powf(L0, 0);
  }
}

static float cal_leg_theta(float phi0, float phi) {
  float theta = 0, alpha = 0;//alpha is the Angle at which the virtual joint motor is turned
  alpha = PI / 2 - phi0;

  if (alpha * phi < 0) {
    theta = ABS(alpha) - ABS(phi);
    if ((alpha > 0) && (phi < 0)) {
      theta *= -1;
    } else {

    }
  } else {
    theta = ABS(alpha) + ABS(phi);
    if ((alpha < 0) && (phi < 0)) {
    } else {
      theta *= -1;
    }
  }
  return theta;
}

void Matrix_multiply(int rows1, int cols1, float matrix1[rows1][cols1],
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

static void state_variable_update(struct Leg *leg_L, struct Leg *leg_R, float phi, float phi_dot) {
  if (leg_L == NULL || leg_R == NULL) {
    return;
  }
  //theta_last
  leg_L->state_variable_feedback.theta_last = leg_L->state_variable_feedback.theta;
  leg_R->state_variable_feedback.theta_last = leg_R->state_variable_feedback.theta;

  //theta
  leg_L->state_variable_feedback.theta =
      cal_leg_theta(leg_L->vmc.forward_kinematics.fk_phi.phi0, phi);
  leg_R->state_variable_feedback.theta =
      cal_leg_theta(leg_R->vmc.forward_kinematics.fk_phi.phi0, phi);

  //theta_ddot
  leg_L->state_variable_feedback.theta_dot_last = leg_L->state_variable_feedback.theta_dot;
  leg_L->state_variable_feedback.theta_dot =
      (leg_L->state_variable_feedback.theta - leg_L->state_variable_feedback.theta_last)
          / (CHASSIS_PERIOD * 0.001f);
  float theta_ddot_raw_L =
      (leg_L->state_variable_feedback.theta_dot - leg_L->state_variable_feedback.theta_dot_last)
          / (CHASSIS_PERIOD * 0.001f);
  updateFilter(&theta_ddot_filter_L, theta_ddot_raw_L);
  leg_L->state_variable_feedback.theta_ddot = getFilteredValue(&theta_ddot_filter_L);

  leg_R->state_variable_feedback.theta_dot_last = leg_R->state_variable_feedback.theta_dot;
  leg_R->state_variable_feedback.theta_dot =
      (leg_R->state_variable_feedback.theta - leg_R->state_variable_feedback.theta_last)
          / (CHASSIS_PERIOD * 0.001f);
  float theta_ddot_raw_R =
      (leg_R->state_variable_feedback.theta_dot - leg_R->state_variable_feedback.theta_dot_last)
          / (CHASSIS_PERIOD * 0.001f);
  updateFilter(&theta_ddot_filter_R, theta_ddot_raw_R);
  leg_R->state_variable_feedback.theta_ddot = getFilteredValue(&theta_ddot_filter_R);

  //x
  if (get_chassis()->chassis_ctrl_info.v_m_per_s != 0
      || get_chassis()->is_chassis_offground == true) {
    leg_L->state_variable_feedback.x = 0;
    leg_R->state_variable_feedback.x = 0;
  } else {
    leg_L->state_variable_feedback.x =
        leg_L->state_variable_feedback.x
            + CHASSIS_PERIOD * 0.001f * ((float) get_wheel_motors()[0].angular_vel * BALANCE_WHEEL_R);
    leg_R->state_variable_feedback.x =
        leg_R->state_variable_feedback.x
            + CHASSIS_PERIOD * 0.001f * ((float) -get_wheel_motors()[1].angular_vel * BALANCE_WHEEL_R);

    VAL_LIMIT(leg_L->state_variable_feedback.x, -0.05, 0.05)
    VAL_LIMIT(leg_R->state_variable_feedback.x, -0.05, 0.05)
  }

  if (get_chassis()->chassis_ctrl_info.v_m_per_s != 0) {
    leg_L->state_variable_feedback.x_dot = leg_L->kalman_result[0];
    leg_R->state_variable_feedback.x_dot = leg_R->kalman_result[0];
  } else {
    leg_L->state_variable_feedback.x_dot = (float) get_wheel_motors()[0].angular_vel * BALANCE_WHEEL_R;
    leg_R->state_variable_feedback.x_dot = (float) -get_wheel_motors()[1].angular_vel * BALANCE_WHEEL_R;
  }
  leg_L->state_variable_feedback.x_dot_last = leg_L->state_variable_feedback.x_dot;
  leg_L->state_variable_feedback.x_ddot =
      (leg_L->state_variable_feedback.x_dot - leg_L->state_variable_feedback.x_dot_last)
          / (CHASSIS_PERIOD * 0.001f);

  leg_R->state_variable_feedback.x_dot_last = leg_R->state_variable_feedback.x_dot;
  leg_R->state_variable_feedback.x_ddot =
      (leg_R->state_variable_feedback.x_dot - leg_R->state_variable_feedback.x_dot_last)
          / (CHASSIS_PERIOD * 0.001f);

  //phi
  leg_L->state_variable_feedback.phi = phi;
  leg_L->state_variable_feedback.phi_dot = phi_dot;

  leg_R->state_variable_feedback.phi = phi;
  leg_R->state_variable_feedback.phi_dot = phi_dot;
}

static void state_variable_set(struct Chassis *chassis) {
  if (chassis == NULL) {
    return;
  }

  chassis->leg_L.state_variable_set_point.x = 0;
  chassis->leg_L.state_variable_set_point.x_dot = chassis->chassis_ctrl_info.v_m_per_s;
  chassis->leg_L.state_variable_set_point.theta = 0.05236f;
  chassis->leg_L.state_variable_set_point.theta_dot = 0;
  chassis->leg_L.state_variable_set_point.phi = 0;
  chassis->leg_L.state_variable_set_point.phi_dot = 0;

  chassis->leg_R.state_variable_set_point.x = 0;
  chassis->leg_R.state_variable_set_point.x_dot = chassis->chassis_ctrl_info.v_m_per_s;
  chassis->leg_R.state_variable_set_point.theta = 0.05236f;
  chassis->leg_R.state_variable_set_point.theta_dot = 0;
  chassis->leg_R.state_variable_set_point.phi = 0;
  chassis->leg_R.state_variable_set_point.phi_dot = 0;
}

static void state_variable_error(struct Leg *leg_L, struct Leg *leg_R) {
  if (leg_L == NULL || leg_R == NULL) {
    return;
  }

  leg_L->state_variable_error.x = leg_L->state_variable_feedback.x - leg_L->state_variable_set_point.x;
  leg_L->state_variable_error.x_dot = leg_L->state_variable_feedback.x_dot - leg_L->state_variable_set_point.x_dot;
  leg_L->state_variable_error.theta = leg_L->state_variable_feedback.theta - leg_L->state_variable_set_point.theta;
  leg_L->state_variable_error.theta_dot =
      leg_L->state_variable_feedback.theta_dot - leg_L->state_variable_set_point.theta_dot;
  leg_L->state_variable_error.phi = leg_L->state_variable_feedback.phi - leg_L->state_variable_set_point.phi;
  leg_L->state_variable_error.phi_dot =
      leg_L->state_variable_feedback.phi_dot - leg_L->state_variable_set_point.phi_dot;

  leg_R->state_variable_error.x = leg_R->state_variable_feedback.x - leg_R->state_variable_set_point.x;
  leg_R->state_variable_error.x_dot = leg_R->state_variable_feedback.x_dot - leg_R->state_variable_set_point.x_dot;
  leg_R->state_variable_error.theta = leg_R->state_variable_feedback.theta - leg_R->state_variable_set_point.theta;
  leg_R->state_variable_error.theta_dot =
      leg_R->state_variable_feedback.theta_dot - leg_R->state_variable_set_point.theta_dot;
  leg_R->state_variable_error.phi = leg_R->state_variable_feedback.phi - leg_R->state_variable_set_point.phi;
  leg_R->state_variable_error.phi_dot =
      leg_R->state_variable_feedback.phi_dot - leg_R->state_variable_set_point.phi_dot;
}

static void state_variable_out(struct Chassis *chassis) {
  if (chassis == NULL) {
    return;
  }
  if (chassis->is_chassis_offground) {
//    for (int i = 0; i < 6; i++) {
//      joint_K_L[i] *= 0.2f;
//      joint_K_R[i] *= 0.2f;
//    }
//    for (int i = 0; i < 6; i++) {
//      wheel_K_L[i] = 0;
//      wheel_K_R[i] = 0;
//    }
//    for (int i = 2; i < 6; i++) {
//      joint_K_L[i] = 0;
//      joint_K_R[i] = 0;
//    }
  }
//
//  if (chassis->chassis_ctrl_mode == CHASSIS_INIT) {
//
//    chassis->leg_L.state_variable_wheel_out.theta = chassis->leg_L.state_variable_error.theta * init_wheel_K_L[0];
//    chassis->leg_L.state_variable_wheel_out.theta_dot =
//        chassis->leg_L.state_variable_error.theta_dot * init_wheel_K_L[1];
//    chassis->leg_L.state_variable_wheel_out.x = chassis->leg_L.state_variable_error.x * init_wheel_K_L[2];
//    chassis->leg_L.state_variable_wheel_out.x_dot = chassis->leg_L.state_variable_error.x_dot * init_wheel_K_L[3];
//    chassis->leg_L.state_variable_wheel_out.phi = chassis->leg_L.state_variable_error.phi * init_wheel_K_L[4];
//    chassis->leg_L.state_variable_wheel_out.phi_dot = chassis->leg_L.state_variable_error.phi_dot * init_wheel_K_L[5];
//
//    chassis->leg_L.state_variable_joint_out.theta = chassis->leg_L.state_variable_error.theta * init_joint_K_L[0];
//    chassis->leg_L.state_variable_joint_out.theta_dot =
//        chassis->leg_L.state_variable_error.theta_dot * init_joint_K_L[1];
//    chassis->leg_L.state_variable_joint_out.x = chassis->leg_L.state_variable_error.x * init_joint_K_L[2];
//    chassis->leg_L.state_variable_joint_out.x_dot = chassis->leg_L.state_variable_error.x_dot * init_joint_K_L[3];
//    chassis->leg_L.state_variable_joint_out.phi = chassis->leg_L.state_variable_error.phi * init_joint_K_L[4];
//    chassis->leg_L.state_variable_joint_out.phi_dot = chassis->leg_L.state_variable_error.phi_dot * init_joint_K_L[5];
//
//    chassis->leg_R.state_variable_wheel_out.theta = chassis->leg_R.state_variable_error.theta * init_wheel_K_R[0];
//    chassis->leg_R.state_variable_wheel_out.theta_dot =
//        chassis->leg_R.state_variable_error.theta_dot * init_wheel_K_R[1];
//    chassis->leg_R.state_variable_wheel_out.x = chassis->leg_R.state_variable_error.x * init_wheel_K_R[2];
//    chassis->leg_R.state_variable_wheel_out.x_dot = chassis->leg_R.state_variable_error.x_dot * init_wheel_K_R[3];
//    chassis->leg_R.state_variable_wheel_out.phi = chassis->leg_R.state_variable_error.phi * init_wheel_K_R[4];
//    chassis->leg_R.state_variable_wheel_out.phi_dot = chassis->leg_R.state_variable_error.phi_dot * init_wheel_K_R[5];
//
//    chassis->leg_R.state_variable_joint_out.theta = chassis->leg_R.state_variable_error.theta * init_joint_K_R[0];
//    chassis->leg_R.state_variable_joint_out.theta_dot =
//        chassis->leg_R.state_variable_error.theta_dot * init_joint_K_R[1];
//    chassis->leg_R.state_variable_joint_out.x = chassis->leg_R.state_variable_error.x * init_joint_K_R[2];
//    chassis->leg_R.state_variable_joint_out.x_dot = chassis->leg_R.state_variable_error.x_dot * init_joint_K_R[3];
//    chassis->leg_R.state_variable_joint_out.phi = chassis->leg_R.state_variable_error.phi * init_joint_K_R[4];
//    chassis->leg_R.state_variable_joint_out.phi_dot = chassis->leg_R.state_variable_error.phi_dot * init_joint_K_R[5];


  chassis->leg_L.state_variable_wheel_out.theta = chassis->leg_L.state_variable_error.theta * wheel_K_L[0];
  chassis->leg_L.state_variable_wheel_out.theta_dot = chassis->leg_L.state_variable_error.theta_dot * wheel_K_L[1];
  chassis->leg_L.state_variable_wheel_out.x = chassis->leg_L.state_variable_error.x * wheel_K_L[2];
  chassis->leg_L.state_variable_wheel_out.x_dot = chassis->leg_L.state_variable_error.x_dot * wheel_K_L[3];
  chassis->leg_L.state_variable_wheel_out.phi = chassis->leg_L.state_variable_error.phi * wheel_K_L[4];
  chassis->leg_L.state_variable_wheel_out.phi_dot = chassis->leg_L.state_variable_error.phi_dot * wheel_K_L[5];

  chassis->leg_L.state_variable_joint_out.theta = chassis->leg_L.state_variable_error.theta * joint_K_L[0];
  chassis->leg_L.state_variable_joint_out.theta_dot = chassis->leg_L.state_variable_error.theta_dot * joint_K_L[1];
  chassis->leg_L.state_variable_joint_out.x = chassis->leg_L.state_variable_error.x * joint_K_L[2];
  chassis->leg_L.state_variable_joint_out.x_dot = chassis->leg_L.state_variable_error.x_dot * joint_K_L[3];
  chassis->leg_L.state_variable_joint_out.phi = chassis->leg_L.state_variable_error.phi * joint_K_L[4];
  chassis->leg_L.state_variable_joint_out.phi_dot = chassis->leg_L.state_variable_error.phi_dot * joint_K_L[5];

  chassis->leg_R.state_variable_wheel_out.theta = chassis->leg_R.state_variable_error.theta * wheel_K_R[0];
  chassis->leg_R.state_variable_wheel_out.theta_dot = chassis->leg_R.state_variable_error.theta_dot * wheel_K_R[1];
  chassis->leg_R.state_variable_wheel_out.x = chassis->leg_R.state_variable_error.x * wheel_K_R[2];
  chassis->leg_R.state_variable_wheel_out.x_dot = chassis->leg_R.state_variable_error.x_dot * wheel_K_R[3];
  chassis->leg_R.state_variable_wheel_out.phi = chassis->leg_R.state_variable_error.phi * wheel_K_R[4];
  chassis->leg_R.state_variable_wheel_out.phi_dot = chassis->leg_R.state_variable_error.phi_dot * wheel_K_R[5];

  chassis->leg_R.state_variable_joint_out.theta = chassis->leg_R.state_variable_error.theta * joint_K_R[0];
  chassis->leg_R.state_variable_joint_out.theta_dot = chassis->leg_R.state_variable_error.theta_dot * joint_K_R[1];
  chassis->leg_R.state_variable_joint_out.x = chassis->leg_R.state_variable_error.x * joint_K_R[2];
  chassis->leg_R.state_variable_joint_out.x_dot = chassis->leg_R.state_variable_error.x_dot * joint_K_R[3];
  chassis->leg_R.state_variable_joint_out.phi = chassis->leg_R.state_variable_error.phi * joint_K_R[4];
  chassis->leg_R.state_variable_joint_out.phi_dot = chassis->leg_R.state_variable_error.phi_dot * joint_K_R[5];

}

/*******************************************************************************
 *                                     LQR                                     *
 *******************************************************************************/

void lqr_ctrl(struct Chassis *chassis) {
  chassis_K_matrix_fitting(chassis->leg_L.vmc.forward_kinematics.fk_L0.L0 * 0.25f, wheel_K_L, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis->leg_L.vmc.forward_kinematics.fk_L0.L0 * 0.25f, joint_K_L, joint_fitting_factor);
  chassis_K_matrix_fitting(chassis->leg_R.vmc.forward_kinematics.fk_L0.L0 * 0.25f, wheel_K_R, wheel_fitting_factor);
  chassis_K_matrix_fitting(chassis->leg_R.vmc.forward_kinematics.fk_L0.L0 * 0.25f, joint_K_R, joint_fitting_factor);
  state_variable_update(&chassis->leg_L,
                        &chassis->leg_R,
                        chassis->imu_reference.pitch_angle,
                        chassis->imu_reference.pitch_gyro);
  state_variable_set(chassis);
  state_variable_error(&chassis->leg_L, &chassis->leg_R);
  state_variable_out(chassis);
}