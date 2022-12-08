#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_1534210039544554190);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7069507310096476943);
void car_H_mod_fun(double *state, double *out_1388971859504496912);
void car_f_fun(double *state, double dt, double *out_3937645064257698984);
void car_F_fun(double *state, double dt, double *out_6633176775052953918);
void car_h_25(double *state, double *unused, double *out_3792083867229189646);
void car_H_25(double *state, double *unused, double *out_939875254275285522);
void car_h_24(double *state, double *unused, double *out_5807376870268347679);
void car_H_24(double *state, double *unused, double *out_7140907934126446948);
void car_h_30(double *state, double *unused, double *out_3190665709615826092);
void car_H_30(double *state, double *unused, double *out_5976815087216331233);
void car_h_26(double *state, double *unused, double *out_1811007558045019553);
void car_H_26(double *state, double *unused, double *out_4681378573149341746);
void car_h_27(double *state, double *unused, double *out_5634116794039885528);
void car_H_27(double *state, double *unused, double *out_3802051775415906322);
void car_h_29(double *state, double *unused, double *out_5791303462391014673);
void car_H_29(double *state, double *unused, double *out_6487046431530723417);
void car_h_28(double *state, double *unused, double *out_5377483845240965275);
void car_H_28(double *state, double *unused, double *out_2993709968523175285);
void car_h_31(double *state, double *unused, double *out_6891394949659689529);
void car_H_31(double *state, double *unused, double *out_909229292398325094);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}