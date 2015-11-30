#include "gyro_pid.h"
#include <Arduino.h>

int p_en = 0, i_en = 0, d_en = 0;
float old_error, cur_error, sum_error;

float k_p, k_i, k_d; 

int timestep = 10; // 1/ Hz

long last_time_run_ms = 0;
long cur_time_ms;

float angle_target;

void gyro_pid_set_gain_enable(int p_enable, int i_enable, int d_enable) {
	p_en = p_enable;
	i_en = i_enable;
	d_en = d_enable;
}

void gyro_pid_set_timestep(int new_timestep) {
	double ratio = (double) new_timestep / (double) timestep;

	timestep = new_timestep;
	k_i *= ratio;
	k_d *= ratio;
}

void gyro_pid_set_gains(float k_p_new, float k_d_new, float k_i_new) {
	k_p = k_p_new;
	k_i = k_i_new * timestep;
	k_d = k_d_new / timestep;
}

void gyro_pid_set_target_angle(float angle_new) {
	angle_target = angle_new;
	sum_error = 0;
}

float determine_min_angle(float angle_target, float angle_cur) {
	float diff = angle_target - angle_cur;
	if(diff < 0) diff += 360;
	
	
	if(diff < 180){
		return diff;
	} 
	return (diff-360);
}

void gyro_pid_update(double angle_cur, float * vel_l, float * vel_r) {
	if((cur_time_ms = millis()) - last_time_run_ms >= timestep) {
		last_time_run_ms = cur_time_ms;

		float pwm_p, pwm_i, pwm_d;

		/* Make error measurements */
		old_error = cur_error;
		cur_error =  determine_min_angle(angle_target, angle_cur);
		sum_error += i_en ? cur_error : 0.0;

		/* Calculate P, I, and D contributions */
		pwm_p = p_en ? k_p * cur_error : 0.0;

		pwm_i = i_en ? k_i * sum_error : 0.0;

		pwm_d = d_en ? k_d * (cur_error - old_error) : 0.0;
			
		float sum_delta = pwm_p + pwm_i + pwm_d;
		*vel_l = sum_delta;
		*vel_r = -sum_delta;
		
		//Serial.println(sum_delta);

		trim_float(vel_l, -22.0, 22.0);
		trim_float(vel_r, -22.0, 22.0);
		//Serial.print(angle_cur);
		//Serial.print('\t');
		//Serial.println(*vel_l);
	}
}