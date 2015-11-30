#ifndef __LIDAR_H__
#define __LIDAR_H__

// #include <sys/mman.h>
// #include <stdio.h>
#include <stdint.h>
// #include <fcntl.h>
#include <Arduino.h>


#define PACKETS_TO_SAVE 2000
#define BYTES_PER_PACKET 22
#define PACKETS_PER_SECOND 450
#define UPDATE_FREQ 5
#define PACKETS_IN_LIST 1000
#define OH_SO_HEXY HEX
#define DATA_PACKET_LENGTH 14
#define DATA_NUM_LINES 45000

typedef struct lidar_packet {
	uint8_t id;
	uint16_t dist[4];
} lidar_packet;

// #define LIDAR_BUF_SIZE 180
// extern lidar_packet lidar_packet_buf[LIDAR_BUF_SIZE];
// extern int lidar_buf_in_ind;
// extern int lidar_buf_out_ind;
// extern int lidar_buf_count;

/* 2-rotation distance scan readings */
typedef struct lidar_dist {
	uint16_t degree;
	uint16_t dist;
	int16_t dist_x;
	int16_t dist_y;
} lidar_dist;

extern lidar_dist lidar_dist_buf_complete[360];
extern int flag_scan_complete;
extern int scan_start_deg;

// void reset_scan_buf();

// int init_lidar_data_save();
// int init_lidar_data_save(int save_count);
// void stop_lidar_save_data();

lidar_dist get_degree(int des_degree);
void init_lidar();
int process_packet();
void loop_lidar();

#endif