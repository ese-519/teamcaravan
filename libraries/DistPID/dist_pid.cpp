#include "dist_pid.h"
#include <Arduino.h>

int dist_p_en = 0, dist_i_en = 0, dist_d_en = 0;
float dist_old_error, dist_cur_error, dist_sum_error;

float dist_k_p, dist_k_i, dist_k_d; 

int dist_timestep = 400; // 1/ Hz

long dist_last_time_run_ms = 0;
long dist_cur_time_ms;

float dist_angle_target;

void dist_pid_set_gain_enable(int p_enable, int i_enable, int d_enable) {
	dist_p_en = p_enable;
	dist_i_en = i_enable;
	dist_d_en = d_enable;
}

void dist_pid_set_timestep(int new_timestep) {
	double ratio = (double) new_timestep / (double) dist_timestep;

	dist_timestep = new_timestep;
	dist_k_i *= ratio;
	dist_k_d *= ratio;
}

void dist_pid_set_gains(float k_p_new, float k_d_new, float k_i_new) {
	dist_k_p = k_p_new;
	dist_k_i = k_i_new * dist_timestep;
	dist_k_d = k_d_new / dist_timestep;
}

void dist_pid_set_target_angle(float angle_new) {
	dist_angle_target = angle_new;
	dist_sum_error = 0;
}

void dist_pid_update(double angle_cur, float * vel_l, float * vel_r, float vel_l_nom, float vel_r_nom) {
	if((dist_cur_time_ms = millis()) - dist_last_time_run_ms >= dist_timestep) {
		dist_last_time_run_ms = dist_cur_time_ms;

		float pwm_p, pwm_i, pwm_d;

		/* Make error measurements */
		dist_old_error = dist_cur_error;
		dist_cur_error =  dist_angle_target - angle_cur;
		dist_sum_error += dist_i_en ? dist_cur_error : 0.0;

		/* Calculate P, I, and D contributions */
		pwm_p = dist_p_en ? dist_k_p * dist_cur_error : 0.0;

		pwm_i = dist_i_en ? dist_k_i * dist_sum_error : 0.0;

		pwm_d = dist_d_en ? dist_k_d * (dist_cur_error - dist_old_error) : 0.0;
			
		float sum_delta = pwm_p + pwm_i + pwm_d;
		*vel_l = vel_l_nom + sum_delta;
		*vel_r = vel_r_nom - sum_delta;
		
		//Serial.println(sum_delta);

		trim_float(vel_l, -22.0, 22.0);
		trim_float(vel_r, -22.0, 22.0);
		//Serial.print(angle_cur);
		//Serial.print('\t');
		//Serial.println(*vel_l);
	}
}