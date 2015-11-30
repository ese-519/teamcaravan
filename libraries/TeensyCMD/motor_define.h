#ifndef __MOTOR_DEFINE_H__
#define __MOTOR_DEFINE_H__

/* File: motor_define.h 
 * Author: Ben Kramer krab@seas.upenn.edu
 * --------------------------------------
 * This file includes the pin assignments for the Teensy 3.1 used to drive the
 * two motors on the HemoGlobetrotter. It also includes constants defining the 
 * max PWM frequency and duty cycle of the motors as well as directional
 * constants and various other defines. Everything except for the motor
 * reference constants should only be used by motor-controller code. The 
 * end user (lookin' at you, Cameron) should be able to control the motors by
 * using MOTOR_LEFT and MOTOR_RIGHT with the appropriate functions
 */

/* Teensy pinout assignments */
#define M1_PWM    22
#define M2_PWM    23

#define M1_IN_A   18
#define M1_IN_B   19
#define M2_IN_A   20
#define M2_IN_B   21

#define M1_ENC_A  2
#define M1_ENC_B  3
#define M2_ENC_A  4
#define M2_ENC_B  5

/* PWM constants */
#define PWM_FREQ  10000
#define PWM_MAX   255

/* Motor driver constants */
#define CW_IN_A   HIGH
#define CW_IN_B   LOW

#define CCW_IN_A  LOW
#define CCW_IN_B  HIGH

#define BRK_IN_A  HIGH
#define BRK_IN_B  HIGH

/* Motor reference constants */
#define MOTOR_LEFT 	1
#define MOTOR_RIGHT 2

#define MOTOR_FWD   1
#define MOTOR_BWD   2
#define MOTOR_BRK   0

/* Control loop settings */
#define UPDATE_RATE 100 
#define MODE_PID 	1
#define MODE_OPEN	2

#define K_P         .017
#define K_I         .00002
#define K_D         .3

#define MAX_VEL      23 /* Max rad/s velocity */

#define MSR_INTVL_MS   5
#define COUNTS_PER_RAD 509.296 // (3200 counts/rev) / (2pi rads / rev)
#define VEL_BUF_SIZE   5

#endif