#ifndef __MPU6050-GY521_H__
#define __MPU6050-GY521_H__

#define DATA_PACKET_LENGTH 14
#define DATA_NUM_LINES 45000


#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <Kalman.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define CLK_SPEED MPU6050_CLOCK_DIV_400

typedef struct imu_packet {
	int16_t ax, ay, az;  //9.81m/s^2 about 16750
	int16_t gx, gy, gz;  // 34664202 = 360 
} imu_packet;

extern Kalman kalmanX; // Create the Kalman instances
extern Kalman kalmanY;

extern int16_t accX, accY, accZ;
extern int16_t gyroX, gyroY, gyroZ;
extern double currentAngle;

extern uint32_t timer;

extern MPU6050 imu;

void init_imu();

void loop_imu();

int imu_open_save_file(int save_count);

int imu_save_data();

int imu_close_save_file();

imu_packet get_data();

double getAccX();
double getAccY();
double getAccZ();
double getGyroX();
double getGyroY();
double getGyroZ();

#endif