#ifndef __GYRO_PID__
#define __GYRO_PID__

#include <Arduino.h>
#include "hemo_funcs.h"

void gyro_pid_set_gain_enable(int p_enable, int i_enable, int d_enable);

void gyro_pid_set_timestep(int new_timestep);

void gyro_pid_set_gains(float k_p_new, float k_d_new, float k_i_new);

void gyro_pid_set_target_angle(float angle_new);

void gyro_pid_update(double angle_cur, float * vel_l, float * vel_r);

#endif