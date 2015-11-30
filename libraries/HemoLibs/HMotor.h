#ifndef __HMOTOR_H__
#define __HMOTOR_H__

#include "motor_define.h"
#include <stdint.h>

class HMotor {
public:

	void initializePins();

  	void initializePWM();

	void updateEncoderValue();

  	/* Returns velocity in rads/sec */
	float estimateVelocity();

	void updateVelocity();

	void updatePID();

	void updateDuty();

	int _fwd_val[2];
	int _rev_val[2];
	int _motor_id;

	/* Pin assignments */
	int _p_pwm;
	int _p_in_a, _p_in_b;
	int _p_enc_a, _p_enc_b;

	/* Private counts */
	uint8_t _cur_enc_val, _old_enc_val;

	/* Motor motion state */
		int _dir;
		float _cur_duty;

		/* Velocity */
		float _vel_buf[VEL_BUF_SIZE];
		float _cur_vel;

	/* PID */
	float _k_p, _k_d, _k_i;
	float _tgt_vel; // target velocity in rads/s
	float _old_error, _cur_error, _sum_error;

	/* Control loop settings */
	int _loop_mode;
	int _stall_flag; /* 0 indicates normal operation, 1 indicates motor stopped due to stall */ 

	HMotor(int motor_id);

	void initMotor();

	/* Low-level output functions */
	/* Set the duty only if we're in open-loop mode */
	void setDuty(float duty);

	void setDirection(int dir);

	void setBrake();

	/* Sets target velocity to [vel] rads/s */
	void setVelocity(float vel);

	void setLoopMode(int mode);

	float getVelocity();

	int isStalled();

	/* updates motor at specified frequency
	 * 1. ensures we aren't stalling, sets flag if we are 
	 * 2. calls proper update rountine */
	void updateMotor();

};

#endif