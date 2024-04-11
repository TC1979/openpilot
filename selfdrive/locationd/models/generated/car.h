#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_1149082796553894114);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8168635988992568031);
void car_H_mod_fun(double *state, double *out_5028546107515860526);
void car_f_fun(double *state, double dt, double *out_8658007590048612807);
void car_F_fun(double *state, double dt, double *out_8587973441833651067);
void car_h_25(double *state, double *unused, double *out_5804801738838160998);
void car_H_25(double *state, double *unused, double *out_924816537098451757);
void car_h_24(double *state, double *unused, double *out_649898787763975145);
void car_H_24(double *state, double *unused, double *out_1252397886508698216);
void car_h_30(double *state, double *unused, double *out_8801194663672072158);
void car_H_30(double *state, double *unused, double *out_1593516421408796870);
void car_h_26(double *state, double *unused, double *out_5363985568793818950);
void car_H_26(double *state, double *unused, double *out_4666319855972507981);
void car_h_27(double *state, double *unused, double *out_178122100467980755);
void car_H_27(double *state, double *unused, double *out_581246890391628041);
void car_h_29(double *state, double *unused, double *out_3957167235751350893);
void car_H_29(double *state, double *unused, double *out_2103747765723189054);
void car_h_28(double *state, double *unused, double *out_3214908151912313440);
void car_H_28(double *state, double *unused, double *out_2978651251346341520);
void car_h_31(double *state, double *unused, double *out_7255433131467044165);
void car_H_31(double *state, double *unused, double *out_5292527958205859457);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}