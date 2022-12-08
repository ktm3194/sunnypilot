#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_6388044244173231044);
void live_err_fun(double *nom_x, double *delta_x, double *out_4169767778236236392);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7822247179386355730);
void live_H_mod_fun(double *state, double *out_6053854872321429887);
void live_f_fun(double *state, double dt, double *out_4280476520384059321);
void live_F_fun(double *state, double dt, double *out_1091854396930247169);
void live_h_4(double *state, double *unused, double *out_3126266448298796195);
void live_H_4(double *state, double *unused, double *out_5043483502661830241);
void live_h_9(double *state, double *unused, double *out_6499883639694337469);
void live_H_9(double *state, double *unused, double *out_2243735432602617229);
void live_h_10(double *state, double *unused, double *out_5227288079474676849);
void live_H_10(double *state, double *unused, double *out_1959436190898681221);
void live_h_12(double *state, double *unused, double *out_2610211743028551379);
void live_H_12(double *state, double *unused, double *out_24027094629868446);
void live_h_35(double *state, double *unused, double *out_8421437788759068072);
void live_H_35(double *state, double *unused, double *out_1676821445289222865);
void live_h_32(double *state, double *unused, double *out_6059960609606399865);
void live_H_32(double *state, double *unused, double *out_7276231858156305098);
void live_h_13(double *state, double *unused, double *out_6003597916542382416);
void live_H_13(double *state, double *unused, double *out_5453712775617636458);
void live_h_14(double *state, double *unused, double *out_6499883639694337469);
void live_H_14(double *state, double *unused, double *out_2243735432602617229);
void live_h_33(double *state, double *unused, double *out_6456224449793825560);
void live_H_33(double *state, double *unused, double *out_1473735559349634739);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}