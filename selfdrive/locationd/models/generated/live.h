#pragma once
#include "rednose/helpers/ekf.h"
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
void live_H(double *in_vec, double *out_4618082393729070417);
void live_err_fun(double *nom_x, double *delta_x, double *out_3060258156276992925);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8341841499130754622);
void live_H_mod_fun(double *state, double *out_7446413034881254145);
void live_f_fun(double *state, double dt, double *out_8758409502537269234);
void live_F_fun(double *state, double dt, double *out_4800631365593745243);
void live_h_4(double *state, double *unused, double *out_6577074584423003357);
void live_H_4(double *state, double *unused, double *out_514416684453624611);
void live_h_9(double *state, double *unused, double *out_5023459176053488323);
void live_H_9(double *state, double *unused, double *out_6772802250810822859);
void live_h_10(double *state, double *unused, double *out_8090985443819547744);
void live_H_10(double *state, double *unused, double *out_39877781824275110);
void live_h_12(double *state, double *unused, double *out_7528243687248204181);
void live_H_12(double *state, double *unused, double *out_4505039723578337184);
void live_h_35(double *state, double *unused, double *out_7696101948494860340);
void live_H_35(double *state, double *unused, double *out_7250602755903350893);
void live_h_32(double *state, double *unused, double *out_7810572331211450407);
void live_H_32(double *state, double *unused, double *out_5394836945598588165);
void live_h_13(double *state, double *unused, double *out_3968372412689241624);
void live_H_13(double *state, double *unused, double *out_1384295900603842982);
void live_h_14(double *state, double *unused, double *out_5023459176053488323);
void live_H_14(double *state, double *unused, double *out_6772802250810822859);
void live_h_33(double *state, double *unused, double *out_4165155633383588936);
void live_H_33(double *state, double *unused, double *out_8045584313167343119);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}