#ifndef VMC_H
#define VMC_H

/*******************************************************************************
 *                            ForwardKinematics                                *
 *******************************************************************************/
struct FKL0 {
  float L0;
  float L0_last;
  float L0_dot;
  float L0_dot_last;
  float L0_ddot;
};

struct FKPhi {//The phi Angle in the five-link
  float phi0;
  float phi1;
  float phi2;
  float phi3;
  float phi4;
};

struct FKPointCoordinates {
  float a_x, a_y;
  float b_x, b_y;
  float c_x, c_y;
  float d_x, d_y;
  float e_x, e_y;
};

struct ForwardKinematics {
  struct FKL0 fk_L0;
  struct FKPhi fk_phi;
  struct FKPointCoordinates fk_point_coordinates;
};

struct VMC {
  struct ForwardKinematics forward_kinematics;
  union {
    float array[2][1];
    struct {
      float w1_fdb;
      float w4_fdb;
    } E;
  } W_fdb;

  union {
    float array[2][1];
    struct {
      float w0_fdb;
      float vy_fdb;
    } E;
  } V_fdb;

  union {
    float array[2][1];
    struct {
      float T1_fdb;
      float T4_fdb;
    } E;
  } T1_T4_fdb;

  union {
    float array[2][1];
    struct {
      float T1_set_point;
      float T4_set_point;
    } E;
  } T1_T4_set_point;

  union {
    float array[2][1];
    struct {
      float Tp_fdb;
      float Fy_fdb;
    } E;
  } Fxy_fdb;

  union {
    float array[2][2];
    struct {
      float Tp_set_point;
      float Fy_set_point;
    } E;
  } Fxy_set_point;

  union {
    float array[2][2];
    struct {
      float x1_1;
      float x1_2;
      float x2_1;
      float x2_2;
    } E;
  } J_w_to_v;

  union {
    float array[2][2];
    struct {
      float x1_1;
      float x1_2;
      float x2_1;
      float x2_2;
    } E;
  } J_F_to_T;

  union {
    float array[2][2];
    struct {
      float x1_1;
      float x1_2;
      float x2_1;
      float x2_2;
    } E;
  } J_T_to_F;
};

/*******************************************************************************
 *                           Inverse Kinematics                                *
 *******************************************************************************/
struct Chassis;
struct ChassisPhysicalConfig;
void vmc_ctrl(struct Chassis *chassis,const struct ChassisPhysicalConfig *chassis_physical_config);

#endif //VMC_H
