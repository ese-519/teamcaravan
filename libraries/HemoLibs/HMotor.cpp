/* Motor.cpp 
 * class containing motor files */

#include "HMotor.h"
#include "Arduino.h"
#include "hemo_funcs.h"
#include "motor_define.h"

using namespace std;

void HMotor::initializePins() {
	pinMode(_p_in_a, OUTPUT);
	pinMode(_p_in_b, OUTPUT);
	pinMode(_p_enc_a, INPUT);
	pinMode(_p_enc_b, INPUT);
}

void HMotor::initializePWM() {
	analogWriteFrequency(_p_pwm, PWM_FREQ);
}


void HMotor::updateEncoderValue() {
	int a_status, b_status;

	_old_enc_val = _cur_enc_val;
    a_status = digitalRead(_p_enc_a);
	b_status = digitalRead(_p_enc_b);
	
	_cur_enc_val = ((b_status == HIGH ? 1 : 0) << 1) + (a_status == HIGH ? 1 : 0);
}

	/* Returns velocity in rads/sec */
float HMotor::estimateVelocity() {
	int count;
	long start_time = millis();

	while(millis() - start_time < MSR_INTVL_MS) {
		updateEncoderValue();
		if((_old_enc_val << 1) ^ (_cur_enc_val << 1)) {
  			count++;
		}
	}

	return ((float)count * (1000 / MSR_INTVL_MS)) / COUNTS_PER_RAD;
}

void HMotor::updateVelocity() {
	float new_vel = estimateVelocity();
	_cur_vel += (new_vel - _vel_buf[0]) / VEL_BUF_SIZE;
	trim_float(&_cur_vel, 0.0, 10000.0);
	shift_into_buf(_vel_buf, VEL_BUF_SIZE, new_vel); 
}

long time_since_updated_ms = 0;
long cur_time;
long motor_timestep = 10;
void HMotor::updatePID() {
//	if((cur_time = millis()) - time_since_updated_ms > motor_timestep) {
//		time_since_updated_ms = cur_time;

		float pwm_p, pwm_i, pwm_d;

		/* Make error measurements */
		_old_error = _cur_error;
		_cur_error =  _tgt_vel - _cur_vel;
		_sum_error += _cur_error;

		/* Calculate P, I, and D contributions */
		pwm_p = _k_p * _cur_error;

		pwm_i = _k_i * _sum_error;

		pwm_d = _k_d * (_cur_error - _old_error);
			
		_cur_duty = pwm_p + pwm_d;
		/*Serial.print(_motor_id);
		Serial.print('\t');
		Serial.print(_tgt_vel);
		Serial.print('\t');
		Serial.print(_cur_vel);
		Serial.print('\t');
		Serial.print(pwm_p);
		Serial.print('\t');
		Serial.print(pwm_d);
		Serial.print('\t');
		*/
		trim_float(&_cur_duty, 0.01, 1.0);
		//Serial.println(_cur_duty);
		/*Serial.println();*/
	//}
}

void HMotor::updateDuty() {
	analogWrite(_p_pwm, _cur_duty * PWM_MAX);
}

HMotor::HMotor(int motor_id) {
	if(motor_id != MOTOR_LEFT && motor_id != MOTOR_RIGHT) {}
	else {
		_motor_id = motor_id;
		_p_pwm = motor_id == MOTOR_LEFT ? M1_PWM : M2_PWM;
		_p_in_a = motor_id == MOTOR_LEFT ? M1_IN_A : M2_IN_A;
		_p_in_b = motor_id == MOTOR_LEFT ? M1_IN_B : M2_IN_B;
		_p_enc_a = motor_id == MOTOR_LEFT ? M1_ENC_A : M2_ENC_A;
		_p_enc_b = motor_id == MOTOR_LEFT ? M1_ENC_B : M2_ENC_B;

		_fwd_val[0] = motor_id == MOTOR_LEFT ? CW_IN_A : CCW_IN_A;
		_fwd_val[1] = motor_id == MOTOR_LEFT ? CW_IN_B : CCW_IN_B;
		_rev_val[0] = motor_id == MOTOR_LEFT ? CCW_IN_A : CW_IN_A;
		_rev_val[1] = motor_id == MOTOR_LEFT ? CCW_IN_B : CW_IN_B;

		_k_p = K_P;
		_k_i = K_I;
		_k_d = K_D;

		_loop_mode = MODE_PID;

		_cur_enc_val = _old_enc_val = 0;
		_cur_duty = 0.5;
		_dir = MOTOR_BRK;

		_cur_vel = _tgt_vel = 0;

		_stall_flag = 0;
	}
}

void HMotor::initMotor() {
	initializePins();
	initializePWM();
	zero_buf(_vel_buf, VEL_BUF_SIZE);
}

/* Low-level output functions */
/* Set the duty only if we're in open-loop mode */
void HMotor::setDuty(float duty) {
	_cur_duty = duty;
	_loop_mode = MODE_OPEN;
}

void HMotor::setDirection(int dir) {
	_cur_duty = 0.5;
	switch(dir) {
		case MOTOR_FWD:
			digitalWrite(_p_in_a, _fwd_val[0]);
			digitalWrite(_p_in_b, _fwd_val[1]);
			_dir = MOTOR_FWD;
			break;
		case MOTOR_BWD:
			digitalWrite(_p_in_a, _rev_val[0]);
			digitalWrite(_p_in_b, _rev_val[1]);
			_dir = MOTOR_BWD;
			break;
		default: break;
	}
}

void HMotor::setBrake() {
	digitalWrite(_p_in_a, BRK_IN_A);
	digitalWrite(_p_in_b, BRK_IN_B);
	_dir = MOTOR_BRK;
	_cur_duty = 0.5;
	//_cur_error = 0.0;
	_tgt_vel = 0;
}

/* Sets target velocity to [vel] rads/s */
void HMotor::setVelocity(float vel) {
	//trim_float(&vel, -MAX_VEL, MAX_VEL);
	_tgt_vel = vel;
	_sum_error = 0.0;
	_loop_mode = MODE_PID;
	_cur_duty = 0;
}

void HMotor::setLoopMode(int mode) {
	switch(mode) {
		case MODE_PID:
		case MODE_OPEN: _loop_mode = mode; break;
		default: break;
	}
}

int HMotor::isStalled() {
	return _stall_flag;
}

float HMotor::getVelocity() {
	return _cur_vel;
}

/* updates motor at specified frequency
 * 1. ensures we aren't stalling, sets flag if we are 
 * 2. calls proper update rountine */
void HMotor::updateMotor() {
	/* Don't run if we're stalled */
	if(_stall_flag) return;

	updateVelocity();

	if(_dir == MOTOR_BRK) {

	}
	else {
		switch(_loop_mode) {
			case MODE_PID: 
				updatePID();
				updateDuty();
				break;

			case MODE_OPEN: 
				break;

			default: break;
		}
	}


	updateDuty();
}





/* todo:
 * 1. have pid error_sum reset after a while of tgt_vel = 0ms
 	--at least be able to reset it with a function
 * 2. add stall detection 
 * 3. add way to go from forward to reverse 
 * 4. maybe have velocity function to accept negative numbers and do away with setDirection? 
 * 5. at some point put in a setGains function for user-input gains or to auto-tune gains?
 * 6. calculate acceleration?
 */