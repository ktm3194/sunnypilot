#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1211190588964287673) {
   out_1211190588964287673[0] = delta_x[0] + nom_x[0];
   out_1211190588964287673[1] = delta_x[1] + nom_x[1];
   out_1211190588964287673[2] = delta_x[2] + nom_x[2];
   out_1211190588964287673[3] = delta_x[3] + nom_x[3];
   out_1211190588964287673[4] = delta_x[4] + nom_x[4];
   out_1211190588964287673[5] = delta_x[5] + nom_x[5];
   out_1211190588964287673[6] = delta_x[6] + nom_x[6];
   out_1211190588964287673[7] = delta_x[7] + nom_x[7];
   out_1211190588964287673[8] = delta_x[8] + nom_x[8];
   out_1211190588964287673[9] = delta_x[9] + nom_x[9];
   out_1211190588964287673[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4668400744020283550) {
   out_4668400744020283550[0] = -nom_x[0] + true_x[0];
   out_4668400744020283550[1] = -nom_x[1] + true_x[1];
   out_4668400744020283550[2] = -nom_x[2] + true_x[2];
   out_4668400744020283550[3] = -nom_x[3] + true_x[3];
   out_4668400744020283550[4] = -nom_x[4] + true_x[4];
   out_4668400744020283550[5] = -nom_x[5] + true_x[5];
   out_4668400744020283550[6] = -nom_x[6] + true_x[6];
   out_4668400744020283550[7] = -nom_x[7] + true_x[7];
   out_4668400744020283550[8] = -nom_x[8] + true_x[8];
   out_4668400744020283550[9] = -nom_x[9] + true_x[9];
   out_4668400744020283550[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_8698613124389067185) {
   out_8698613124389067185[0] = 1.0;
   out_8698613124389067185[1] = 0;
   out_8698613124389067185[2] = 0;
   out_8698613124389067185[3] = 0;
   out_8698613124389067185[4] = 0;
   out_8698613124389067185[5] = 0;
   out_8698613124389067185[6] = 0;
   out_8698613124389067185[7] = 0;
   out_8698613124389067185[8] = 0;
   out_8698613124389067185[9] = 0;
   out_8698613124389067185[10] = 0;
   out_8698613124389067185[11] = 0;
   out_8698613124389067185[12] = 1.0;
   out_8698613124389067185[13] = 0;
   out_8698613124389067185[14] = 0;
   out_8698613124389067185[15] = 0;
   out_8698613124389067185[16] = 0;
   out_8698613124389067185[17] = 0;
   out_8698613124389067185[18] = 0;
   out_8698613124389067185[19] = 0;
   out_8698613124389067185[20] = 0;
   out_8698613124389067185[21] = 0;
   out_8698613124389067185[22] = 0;
   out_8698613124389067185[23] = 0;
   out_8698613124389067185[24] = 1.0;
   out_8698613124389067185[25] = 0;
   out_8698613124389067185[26] = 0;
   out_8698613124389067185[27] = 0;
   out_8698613124389067185[28] = 0;
   out_8698613124389067185[29] = 0;
   out_8698613124389067185[30] = 0;
   out_8698613124389067185[31] = 0;
   out_8698613124389067185[32] = 0;
   out_8698613124389067185[33] = 0;
   out_8698613124389067185[34] = 0;
   out_8698613124389067185[35] = 0;
   out_8698613124389067185[36] = 1.0;
   out_8698613124389067185[37] = 0;
   out_8698613124389067185[38] = 0;
   out_8698613124389067185[39] = 0;
   out_8698613124389067185[40] = 0;
   out_8698613124389067185[41] = 0;
   out_8698613124389067185[42] = 0;
   out_8698613124389067185[43] = 0;
   out_8698613124389067185[44] = 0;
   out_8698613124389067185[45] = 0;
   out_8698613124389067185[46] = 0;
   out_8698613124389067185[47] = 0;
   out_8698613124389067185[48] = 1.0;
   out_8698613124389067185[49] = 0;
   out_8698613124389067185[50] = 0;
   out_8698613124389067185[51] = 0;
   out_8698613124389067185[52] = 0;
   out_8698613124389067185[53] = 0;
   out_8698613124389067185[54] = 0;
   out_8698613124389067185[55] = 0;
   out_8698613124389067185[56] = 0;
   out_8698613124389067185[57] = 0;
   out_8698613124389067185[58] = 0;
   out_8698613124389067185[59] = 0;
   out_8698613124389067185[60] = 1.0;
   out_8698613124389067185[61] = 0;
   out_8698613124389067185[62] = 0;
   out_8698613124389067185[63] = 0;
   out_8698613124389067185[64] = 0;
   out_8698613124389067185[65] = 0;
   out_8698613124389067185[66] = 0;
   out_8698613124389067185[67] = 0;
   out_8698613124389067185[68] = 0;
   out_8698613124389067185[69] = 0;
   out_8698613124389067185[70] = 0;
   out_8698613124389067185[71] = 0;
   out_8698613124389067185[72] = 1.0;
   out_8698613124389067185[73] = 0;
   out_8698613124389067185[74] = 0;
   out_8698613124389067185[75] = 0;
   out_8698613124389067185[76] = 0;
   out_8698613124389067185[77] = 0;
   out_8698613124389067185[78] = 0;
   out_8698613124389067185[79] = 0;
   out_8698613124389067185[80] = 0;
   out_8698613124389067185[81] = 0;
   out_8698613124389067185[82] = 0;
   out_8698613124389067185[83] = 0;
   out_8698613124389067185[84] = 1.0;
   out_8698613124389067185[85] = 0;
   out_8698613124389067185[86] = 0;
   out_8698613124389067185[87] = 0;
   out_8698613124389067185[88] = 0;
   out_8698613124389067185[89] = 0;
   out_8698613124389067185[90] = 0;
   out_8698613124389067185[91] = 0;
   out_8698613124389067185[92] = 0;
   out_8698613124389067185[93] = 0;
   out_8698613124389067185[94] = 0;
   out_8698613124389067185[95] = 0;
   out_8698613124389067185[96] = 1.0;
   out_8698613124389067185[97] = 0;
   out_8698613124389067185[98] = 0;
   out_8698613124389067185[99] = 0;
   out_8698613124389067185[100] = 0;
   out_8698613124389067185[101] = 0;
   out_8698613124389067185[102] = 0;
   out_8698613124389067185[103] = 0;
   out_8698613124389067185[104] = 0;
   out_8698613124389067185[105] = 0;
   out_8698613124389067185[106] = 0;
   out_8698613124389067185[107] = 0;
   out_8698613124389067185[108] = 1.0;
   out_8698613124389067185[109] = 0;
   out_8698613124389067185[110] = 0;
   out_8698613124389067185[111] = 0;
   out_8698613124389067185[112] = 0;
   out_8698613124389067185[113] = 0;
   out_8698613124389067185[114] = 0;
   out_8698613124389067185[115] = 0;
   out_8698613124389067185[116] = 0;
   out_8698613124389067185[117] = 0;
   out_8698613124389067185[118] = 0;
   out_8698613124389067185[119] = 0;
   out_8698613124389067185[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_6792734176487019099) {
   out_6792734176487019099[0] = dt*state[3] + state[0];
   out_6792734176487019099[1] = dt*state[4] + state[1];
   out_6792734176487019099[2] = dt*state[5] + state[2];
   out_6792734176487019099[3] = state[3];
   out_6792734176487019099[4] = state[4];
   out_6792734176487019099[5] = state[5];
   out_6792734176487019099[6] = dt*state[7] + state[6];
   out_6792734176487019099[7] = dt*state[8] + state[7];
   out_6792734176487019099[8] = state[8];
   out_6792734176487019099[9] = state[9];
   out_6792734176487019099[10] = state[10];
}
void F_fun(double *state, double dt, double *out_9185204376326532195) {
   out_9185204376326532195[0] = 1;
   out_9185204376326532195[1] = 0;
   out_9185204376326532195[2] = 0;
   out_9185204376326532195[3] = dt;
   out_9185204376326532195[4] = 0;
   out_9185204376326532195[5] = 0;
   out_9185204376326532195[6] = 0;
   out_9185204376326532195[7] = 0;
   out_9185204376326532195[8] = 0;
   out_9185204376326532195[9] = 0;
   out_9185204376326532195[10] = 0;
   out_9185204376326532195[11] = 0;
   out_9185204376326532195[12] = 1;
   out_9185204376326532195[13] = 0;
   out_9185204376326532195[14] = 0;
   out_9185204376326532195[15] = dt;
   out_9185204376326532195[16] = 0;
   out_9185204376326532195[17] = 0;
   out_9185204376326532195[18] = 0;
   out_9185204376326532195[19] = 0;
   out_9185204376326532195[20] = 0;
   out_9185204376326532195[21] = 0;
   out_9185204376326532195[22] = 0;
   out_9185204376326532195[23] = 0;
   out_9185204376326532195[24] = 1;
   out_9185204376326532195[25] = 0;
   out_9185204376326532195[26] = 0;
   out_9185204376326532195[27] = dt;
   out_9185204376326532195[28] = 0;
   out_9185204376326532195[29] = 0;
   out_9185204376326532195[30] = 0;
   out_9185204376326532195[31] = 0;
   out_9185204376326532195[32] = 0;
   out_9185204376326532195[33] = 0;
   out_9185204376326532195[34] = 0;
   out_9185204376326532195[35] = 0;
   out_9185204376326532195[36] = 1;
   out_9185204376326532195[37] = 0;
   out_9185204376326532195[38] = 0;
   out_9185204376326532195[39] = 0;
   out_9185204376326532195[40] = 0;
   out_9185204376326532195[41] = 0;
   out_9185204376326532195[42] = 0;
   out_9185204376326532195[43] = 0;
   out_9185204376326532195[44] = 0;
   out_9185204376326532195[45] = 0;
   out_9185204376326532195[46] = 0;
   out_9185204376326532195[47] = 0;
   out_9185204376326532195[48] = 1;
   out_9185204376326532195[49] = 0;
   out_9185204376326532195[50] = 0;
   out_9185204376326532195[51] = 0;
   out_9185204376326532195[52] = 0;
   out_9185204376326532195[53] = 0;
   out_9185204376326532195[54] = 0;
   out_9185204376326532195[55] = 0;
   out_9185204376326532195[56] = 0;
   out_9185204376326532195[57] = 0;
   out_9185204376326532195[58] = 0;
   out_9185204376326532195[59] = 0;
   out_9185204376326532195[60] = 1;
   out_9185204376326532195[61] = 0;
   out_9185204376326532195[62] = 0;
   out_9185204376326532195[63] = 0;
   out_9185204376326532195[64] = 0;
   out_9185204376326532195[65] = 0;
   out_9185204376326532195[66] = 0;
   out_9185204376326532195[67] = 0;
   out_9185204376326532195[68] = 0;
   out_9185204376326532195[69] = 0;
   out_9185204376326532195[70] = 0;
   out_9185204376326532195[71] = 0;
   out_9185204376326532195[72] = 1;
   out_9185204376326532195[73] = dt;
   out_9185204376326532195[74] = 0;
   out_9185204376326532195[75] = 0;
   out_9185204376326532195[76] = 0;
   out_9185204376326532195[77] = 0;
   out_9185204376326532195[78] = 0;
   out_9185204376326532195[79] = 0;
   out_9185204376326532195[80] = 0;
   out_9185204376326532195[81] = 0;
   out_9185204376326532195[82] = 0;
   out_9185204376326532195[83] = 0;
   out_9185204376326532195[84] = 1;
   out_9185204376326532195[85] = dt;
   out_9185204376326532195[86] = 0;
   out_9185204376326532195[87] = 0;
   out_9185204376326532195[88] = 0;
   out_9185204376326532195[89] = 0;
   out_9185204376326532195[90] = 0;
   out_9185204376326532195[91] = 0;
   out_9185204376326532195[92] = 0;
   out_9185204376326532195[93] = 0;
   out_9185204376326532195[94] = 0;
   out_9185204376326532195[95] = 0;
   out_9185204376326532195[96] = 1;
   out_9185204376326532195[97] = 0;
   out_9185204376326532195[98] = 0;
   out_9185204376326532195[99] = 0;
   out_9185204376326532195[100] = 0;
   out_9185204376326532195[101] = 0;
   out_9185204376326532195[102] = 0;
   out_9185204376326532195[103] = 0;
   out_9185204376326532195[104] = 0;
   out_9185204376326532195[105] = 0;
   out_9185204376326532195[106] = 0;
   out_9185204376326532195[107] = 0;
   out_9185204376326532195[108] = 1;
   out_9185204376326532195[109] = 0;
   out_9185204376326532195[110] = 0;
   out_9185204376326532195[111] = 0;
   out_9185204376326532195[112] = 0;
   out_9185204376326532195[113] = 0;
   out_9185204376326532195[114] = 0;
   out_9185204376326532195[115] = 0;
   out_9185204376326532195[116] = 0;
   out_9185204376326532195[117] = 0;
   out_9185204376326532195[118] = 0;
   out_9185204376326532195[119] = 0;
   out_9185204376326532195[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_4032568693130835250) {
   out_4032568693130835250[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_2386989891395922833) {
   out_2386989891395922833[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2386989891395922833[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2386989891395922833[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2386989891395922833[3] = 0;
   out_2386989891395922833[4] = 0;
   out_2386989891395922833[5] = 0;
   out_2386989891395922833[6] = 1;
   out_2386989891395922833[7] = 0;
   out_2386989891395922833[8] = 0;
   out_2386989891395922833[9] = 0;
   out_2386989891395922833[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_9060966407354429248) {
   out_9060966407354429248[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_7747868725777184499) {
   out_7747868725777184499[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7747868725777184499[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7747868725777184499[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_7747868725777184499[3] = 0;
   out_7747868725777184499[4] = 0;
   out_7747868725777184499[5] = 0;
   out_7747868725777184499[6] = 1;
   out_7747868725777184499[7] = 0;
   out_7747868725777184499[8] = 0;
   out_7747868725777184499[9] = 1;
   out_7747868725777184499[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_8580108849608211157) {
   out_8580108849608211157[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_8554384542770459567) {
   out_8554384542770459567[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8554384542770459567[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8554384542770459567[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8554384542770459567[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8554384542770459567[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8554384542770459567[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8554384542770459567[6] = 0;
   out_8554384542770459567[7] = 1;
   out_8554384542770459567[8] = 0;
   out_8554384542770459567[9] = 0;
   out_8554384542770459567[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_8580108849608211157) {
   out_8580108849608211157[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_8554384542770459567) {
   out_8554384542770459567[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8554384542770459567[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8554384542770459567[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8554384542770459567[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8554384542770459567[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8554384542770459567[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8554384542770459567[6] = 0;
   out_8554384542770459567[7] = 1;
   out_8554384542770459567[8] = 0;
   out_8554384542770459567[9] = 0;
   out_8554384542770459567[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_1211190588964287673) {
  err_fun(nom_x, delta_x, out_1211190588964287673);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4668400744020283550) {
  inv_err_fun(nom_x, true_x, out_4668400744020283550);
}
void gnss_H_mod_fun(double *state, double *out_8698613124389067185) {
  H_mod_fun(state, out_8698613124389067185);
}
void gnss_f_fun(double *state, double dt, double *out_6792734176487019099) {
  f_fun(state,  dt, out_6792734176487019099);
}
void gnss_F_fun(double *state, double dt, double *out_9185204376326532195) {
  F_fun(state,  dt, out_9185204376326532195);
}
void gnss_h_6(double *state, double *sat_pos, double *out_4032568693130835250) {
  h_6(state, sat_pos, out_4032568693130835250);
}
void gnss_H_6(double *state, double *sat_pos, double *out_2386989891395922833) {
  H_6(state, sat_pos, out_2386989891395922833);
}
void gnss_h_20(double *state, double *sat_pos, double *out_9060966407354429248) {
  h_20(state, sat_pos, out_9060966407354429248);
}
void gnss_H_20(double *state, double *sat_pos, double *out_7747868725777184499) {
  H_20(state, sat_pos, out_7747868725777184499);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8580108849608211157) {
  h_7(state, sat_pos_vel, out_8580108849608211157);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8554384542770459567) {
  H_7(state, sat_pos_vel, out_8554384542770459567);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8580108849608211157) {
  h_21(state, sat_pos_vel, out_8580108849608211157);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8554384542770459567) {
  H_21(state, sat_pos_vel, out_8554384542770459567);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
