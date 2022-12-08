#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_1211190588964287673);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4668400744020283550);
void gnss_H_mod_fun(double *state, double *out_8698613124389067185);
void gnss_f_fun(double *state, double dt, double *out_6792734176487019099);
void gnss_F_fun(double *state, double dt, double *out_9185204376326532195);
void gnss_h_6(double *state, double *sat_pos, double *out_4032568693130835250);
void gnss_H_6(double *state, double *sat_pos, double *out_2386989891395922833);
void gnss_h_20(double *state, double *sat_pos, double *out_9060966407354429248);
void gnss_H_20(double *state, double *sat_pos, double *out_7747868725777184499);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8580108849608211157);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8554384542770459567);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8580108849608211157);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8554384542770459567);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}