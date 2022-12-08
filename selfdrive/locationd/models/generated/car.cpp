#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1534210039544554190) {
   out_1534210039544554190[0] = delta_x[0] + nom_x[0];
   out_1534210039544554190[1] = delta_x[1] + nom_x[1];
   out_1534210039544554190[2] = delta_x[2] + nom_x[2];
   out_1534210039544554190[3] = delta_x[3] + nom_x[3];
   out_1534210039544554190[4] = delta_x[4] + nom_x[4];
   out_1534210039544554190[5] = delta_x[5] + nom_x[5];
   out_1534210039544554190[6] = delta_x[6] + nom_x[6];
   out_1534210039544554190[7] = delta_x[7] + nom_x[7];
   out_1534210039544554190[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7069507310096476943) {
   out_7069507310096476943[0] = -nom_x[0] + true_x[0];
   out_7069507310096476943[1] = -nom_x[1] + true_x[1];
   out_7069507310096476943[2] = -nom_x[2] + true_x[2];
   out_7069507310096476943[3] = -nom_x[3] + true_x[3];
   out_7069507310096476943[4] = -nom_x[4] + true_x[4];
   out_7069507310096476943[5] = -nom_x[5] + true_x[5];
   out_7069507310096476943[6] = -nom_x[6] + true_x[6];
   out_7069507310096476943[7] = -nom_x[7] + true_x[7];
   out_7069507310096476943[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1388971859504496912) {
   out_1388971859504496912[0] = 1.0;
   out_1388971859504496912[1] = 0;
   out_1388971859504496912[2] = 0;
   out_1388971859504496912[3] = 0;
   out_1388971859504496912[4] = 0;
   out_1388971859504496912[5] = 0;
   out_1388971859504496912[6] = 0;
   out_1388971859504496912[7] = 0;
   out_1388971859504496912[8] = 0;
   out_1388971859504496912[9] = 0;
   out_1388971859504496912[10] = 1.0;
   out_1388971859504496912[11] = 0;
   out_1388971859504496912[12] = 0;
   out_1388971859504496912[13] = 0;
   out_1388971859504496912[14] = 0;
   out_1388971859504496912[15] = 0;
   out_1388971859504496912[16] = 0;
   out_1388971859504496912[17] = 0;
   out_1388971859504496912[18] = 0;
   out_1388971859504496912[19] = 0;
   out_1388971859504496912[20] = 1.0;
   out_1388971859504496912[21] = 0;
   out_1388971859504496912[22] = 0;
   out_1388971859504496912[23] = 0;
   out_1388971859504496912[24] = 0;
   out_1388971859504496912[25] = 0;
   out_1388971859504496912[26] = 0;
   out_1388971859504496912[27] = 0;
   out_1388971859504496912[28] = 0;
   out_1388971859504496912[29] = 0;
   out_1388971859504496912[30] = 1.0;
   out_1388971859504496912[31] = 0;
   out_1388971859504496912[32] = 0;
   out_1388971859504496912[33] = 0;
   out_1388971859504496912[34] = 0;
   out_1388971859504496912[35] = 0;
   out_1388971859504496912[36] = 0;
   out_1388971859504496912[37] = 0;
   out_1388971859504496912[38] = 0;
   out_1388971859504496912[39] = 0;
   out_1388971859504496912[40] = 1.0;
   out_1388971859504496912[41] = 0;
   out_1388971859504496912[42] = 0;
   out_1388971859504496912[43] = 0;
   out_1388971859504496912[44] = 0;
   out_1388971859504496912[45] = 0;
   out_1388971859504496912[46] = 0;
   out_1388971859504496912[47] = 0;
   out_1388971859504496912[48] = 0;
   out_1388971859504496912[49] = 0;
   out_1388971859504496912[50] = 1.0;
   out_1388971859504496912[51] = 0;
   out_1388971859504496912[52] = 0;
   out_1388971859504496912[53] = 0;
   out_1388971859504496912[54] = 0;
   out_1388971859504496912[55] = 0;
   out_1388971859504496912[56] = 0;
   out_1388971859504496912[57] = 0;
   out_1388971859504496912[58] = 0;
   out_1388971859504496912[59] = 0;
   out_1388971859504496912[60] = 1.0;
   out_1388971859504496912[61] = 0;
   out_1388971859504496912[62] = 0;
   out_1388971859504496912[63] = 0;
   out_1388971859504496912[64] = 0;
   out_1388971859504496912[65] = 0;
   out_1388971859504496912[66] = 0;
   out_1388971859504496912[67] = 0;
   out_1388971859504496912[68] = 0;
   out_1388971859504496912[69] = 0;
   out_1388971859504496912[70] = 1.0;
   out_1388971859504496912[71] = 0;
   out_1388971859504496912[72] = 0;
   out_1388971859504496912[73] = 0;
   out_1388971859504496912[74] = 0;
   out_1388971859504496912[75] = 0;
   out_1388971859504496912[76] = 0;
   out_1388971859504496912[77] = 0;
   out_1388971859504496912[78] = 0;
   out_1388971859504496912[79] = 0;
   out_1388971859504496912[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3937645064257698984) {
   out_3937645064257698984[0] = state[0];
   out_3937645064257698984[1] = state[1];
   out_3937645064257698984[2] = state[2];
   out_3937645064257698984[3] = state[3];
   out_3937645064257698984[4] = state[4];
   out_3937645064257698984[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3937645064257698984[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3937645064257698984[7] = state[7];
   out_3937645064257698984[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6633176775052953918) {
   out_6633176775052953918[0] = 1;
   out_6633176775052953918[1] = 0;
   out_6633176775052953918[2] = 0;
   out_6633176775052953918[3] = 0;
   out_6633176775052953918[4] = 0;
   out_6633176775052953918[5] = 0;
   out_6633176775052953918[6] = 0;
   out_6633176775052953918[7] = 0;
   out_6633176775052953918[8] = 0;
   out_6633176775052953918[9] = 0;
   out_6633176775052953918[10] = 1;
   out_6633176775052953918[11] = 0;
   out_6633176775052953918[12] = 0;
   out_6633176775052953918[13] = 0;
   out_6633176775052953918[14] = 0;
   out_6633176775052953918[15] = 0;
   out_6633176775052953918[16] = 0;
   out_6633176775052953918[17] = 0;
   out_6633176775052953918[18] = 0;
   out_6633176775052953918[19] = 0;
   out_6633176775052953918[20] = 1;
   out_6633176775052953918[21] = 0;
   out_6633176775052953918[22] = 0;
   out_6633176775052953918[23] = 0;
   out_6633176775052953918[24] = 0;
   out_6633176775052953918[25] = 0;
   out_6633176775052953918[26] = 0;
   out_6633176775052953918[27] = 0;
   out_6633176775052953918[28] = 0;
   out_6633176775052953918[29] = 0;
   out_6633176775052953918[30] = 1;
   out_6633176775052953918[31] = 0;
   out_6633176775052953918[32] = 0;
   out_6633176775052953918[33] = 0;
   out_6633176775052953918[34] = 0;
   out_6633176775052953918[35] = 0;
   out_6633176775052953918[36] = 0;
   out_6633176775052953918[37] = 0;
   out_6633176775052953918[38] = 0;
   out_6633176775052953918[39] = 0;
   out_6633176775052953918[40] = 1;
   out_6633176775052953918[41] = 0;
   out_6633176775052953918[42] = 0;
   out_6633176775052953918[43] = 0;
   out_6633176775052953918[44] = 0;
   out_6633176775052953918[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6633176775052953918[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6633176775052953918[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6633176775052953918[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6633176775052953918[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6633176775052953918[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6633176775052953918[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6633176775052953918[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6633176775052953918[53] = -9.8000000000000007*dt;
   out_6633176775052953918[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6633176775052953918[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6633176775052953918[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6633176775052953918[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6633176775052953918[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6633176775052953918[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6633176775052953918[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6633176775052953918[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6633176775052953918[62] = 0;
   out_6633176775052953918[63] = 0;
   out_6633176775052953918[64] = 0;
   out_6633176775052953918[65] = 0;
   out_6633176775052953918[66] = 0;
   out_6633176775052953918[67] = 0;
   out_6633176775052953918[68] = 0;
   out_6633176775052953918[69] = 0;
   out_6633176775052953918[70] = 1;
   out_6633176775052953918[71] = 0;
   out_6633176775052953918[72] = 0;
   out_6633176775052953918[73] = 0;
   out_6633176775052953918[74] = 0;
   out_6633176775052953918[75] = 0;
   out_6633176775052953918[76] = 0;
   out_6633176775052953918[77] = 0;
   out_6633176775052953918[78] = 0;
   out_6633176775052953918[79] = 0;
   out_6633176775052953918[80] = 1;
}
void h_25(double *state, double *unused, double *out_3792083867229189646) {
   out_3792083867229189646[0] = state[6];
}
void H_25(double *state, double *unused, double *out_939875254275285522) {
   out_939875254275285522[0] = 0;
   out_939875254275285522[1] = 0;
   out_939875254275285522[2] = 0;
   out_939875254275285522[3] = 0;
   out_939875254275285522[4] = 0;
   out_939875254275285522[5] = 0;
   out_939875254275285522[6] = 1;
   out_939875254275285522[7] = 0;
   out_939875254275285522[8] = 0;
}
void h_24(double *state, double *unused, double *out_5807376870268347679) {
   out_5807376870268347679[0] = state[4];
   out_5807376870268347679[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7140907934126446948) {
   out_7140907934126446948[0] = 0;
   out_7140907934126446948[1] = 0;
   out_7140907934126446948[2] = 0;
   out_7140907934126446948[3] = 0;
   out_7140907934126446948[4] = 1;
   out_7140907934126446948[5] = 0;
   out_7140907934126446948[6] = 0;
   out_7140907934126446948[7] = 0;
   out_7140907934126446948[8] = 0;
   out_7140907934126446948[9] = 0;
   out_7140907934126446948[10] = 0;
   out_7140907934126446948[11] = 0;
   out_7140907934126446948[12] = 0;
   out_7140907934126446948[13] = 0;
   out_7140907934126446948[14] = 1;
   out_7140907934126446948[15] = 0;
   out_7140907934126446948[16] = 0;
   out_7140907934126446948[17] = 0;
}
void h_30(double *state, double *unused, double *out_3190665709615826092) {
   out_3190665709615826092[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5976815087216331233) {
   out_5976815087216331233[0] = 0;
   out_5976815087216331233[1] = 0;
   out_5976815087216331233[2] = 0;
   out_5976815087216331233[3] = 0;
   out_5976815087216331233[4] = 1;
   out_5976815087216331233[5] = 0;
   out_5976815087216331233[6] = 0;
   out_5976815087216331233[7] = 0;
   out_5976815087216331233[8] = 0;
}
void h_26(double *state, double *unused, double *out_1811007558045019553) {
   out_1811007558045019553[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4681378573149341746) {
   out_4681378573149341746[0] = 0;
   out_4681378573149341746[1] = 0;
   out_4681378573149341746[2] = 0;
   out_4681378573149341746[3] = 0;
   out_4681378573149341746[4] = 0;
   out_4681378573149341746[5] = 0;
   out_4681378573149341746[6] = 0;
   out_4681378573149341746[7] = 1;
   out_4681378573149341746[8] = 0;
}
void h_27(double *state, double *unused, double *out_5634116794039885528) {
   out_5634116794039885528[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3802051775415906322) {
   out_3802051775415906322[0] = 0;
   out_3802051775415906322[1] = 0;
   out_3802051775415906322[2] = 0;
   out_3802051775415906322[3] = 1;
   out_3802051775415906322[4] = 0;
   out_3802051775415906322[5] = 0;
   out_3802051775415906322[6] = 0;
   out_3802051775415906322[7] = 0;
   out_3802051775415906322[8] = 0;
}
void h_29(double *state, double *unused, double *out_5791303462391014673) {
   out_5791303462391014673[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6487046431530723417) {
   out_6487046431530723417[0] = 0;
   out_6487046431530723417[1] = 1;
   out_6487046431530723417[2] = 0;
   out_6487046431530723417[3] = 0;
   out_6487046431530723417[4] = 0;
   out_6487046431530723417[5] = 0;
   out_6487046431530723417[6] = 0;
   out_6487046431530723417[7] = 0;
   out_6487046431530723417[8] = 0;
}
void h_28(double *state, double *unused, double *out_5377483845240965275) {
   out_5377483845240965275[0] = state[0];
}
void H_28(double *state, double *unused, double *out_2993709968523175285) {
   out_2993709968523175285[0] = 1;
   out_2993709968523175285[1] = 0;
   out_2993709968523175285[2] = 0;
   out_2993709968523175285[3] = 0;
   out_2993709968523175285[4] = 0;
   out_2993709968523175285[5] = 0;
   out_2993709968523175285[6] = 0;
   out_2993709968523175285[7] = 0;
   out_2993709968523175285[8] = 0;
}
void h_31(double *state, double *unused, double *out_6891394949659689529) {
   out_6891394949659689529[0] = state[8];
}
void H_31(double *state, double *unused, double *out_909229292398325094) {
   out_909229292398325094[0] = 0;
   out_909229292398325094[1] = 0;
   out_909229292398325094[2] = 0;
   out_909229292398325094[3] = 0;
   out_909229292398325094[4] = 0;
   out_909229292398325094[5] = 0;
   out_909229292398325094[6] = 0;
   out_909229292398325094[7] = 0;
   out_909229292398325094[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_1534210039544554190) {
  err_fun(nom_x, delta_x, out_1534210039544554190);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7069507310096476943) {
  inv_err_fun(nom_x, true_x, out_7069507310096476943);
}
void car_H_mod_fun(double *state, double *out_1388971859504496912) {
  H_mod_fun(state, out_1388971859504496912);
}
void car_f_fun(double *state, double dt, double *out_3937645064257698984) {
  f_fun(state,  dt, out_3937645064257698984);
}
void car_F_fun(double *state, double dt, double *out_6633176775052953918) {
  F_fun(state,  dt, out_6633176775052953918);
}
void car_h_25(double *state, double *unused, double *out_3792083867229189646) {
  h_25(state, unused, out_3792083867229189646);
}
void car_H_25(double *state, double *unused, double *out_939875254275285522) {
  H_25(state, unused, out_939875254275285522);
}
void car_h_24(double *state, double *unused, double *out_5807376870268347679) {
  h_24(state, unused, out_5807376870268347679);
}
void car_H_24(double *state, double *unused, double *out_7140907934126446948) {
  H_24(state, unused, out_7140907934126446948);
}
void car_h_30(double *state, double *unused, double *out_3190665709615826092) {
  h_30(state, unused, out_3190665709615826092);
}
void car_H_30(double *state, double *unused, double *out_5976815087216331233) {
  H_30(state, unused, out_5976815087216331233);
}
void car_h_26(double *state, double *unused, double *out_1811007558045019553) {
  h_26(state, unused, out_1811007558045019553);
}
void car_H_26(double *state, double *unused, double *out_4681378573149341746) {
  H_26(state, unused, out_4681378573149341746);
}
void car_h_27(double *state, double *unused, double *out_5634116794039885528) {
  h_27(state, unused, out_5634116794039885528);
}
void car_H_27(double *state, double *unused, double *out_3802051775415906322) {
  H_27(state, unused, out_3802051775415906322);
}
void car_h_29(double *state, double *unused, double *out_5791303462391014673) {
  h_29(state, unused, out_5791303462391014673);
}
void car_H_29(double *state, double *unused, double *out_6487046431530723417) {
  H_29(state, unused, out_6487046431530723417);
}
void car_h_28(double *state, double *unused, double *out_5377483845240965275) {
  h_28(state, unused, out_5377483845240965275);
}
void car_H_28(double *state, double *unused, double *out_2993709968523175285) {
  H_28(state, unused, out_2993709968523175285);
}
void car_h_31(double *state, double *unused, double *out_6891394949659689529) {
  h_31(state, unused, out_6891394949659689529);
}
void car_H_31(double *state, double *unused, double *out_909229292398325094) {
  H_31(state, unused, out_909229292398325094);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
