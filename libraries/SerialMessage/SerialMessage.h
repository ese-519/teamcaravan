#ifndef __SMSG_H__
#define __SMSG_H__

#define PACKET_SIZE 10
#define PACKET_START 2   // STX
#define PACKET_END 3     // ETX

#define INFO_MSG 4
#define ACK_MSG 0x05
#define TURN_MSG 6
#define FWD_MSG 7
#define BWD_MSG 8
#define BRK_MSG 9

#define INFO_OBS 5
#define INFO_HLWY 6
#define INFO_HAZ_RNG 7
#define INFO_WALL_RNG 8

#define DIR_LEFT 1
#define DIR_RIGHT 2

#define RPI_SERIAL Serial2

bool is_numeric(unsigned char c);

int c_to_i(unsigned char c);

unsigned char i_to_c(int i);

char* i_to_s(int i);

int s_to_i(unsigned char* s, int l);

void make_packet(unsigned char *buf, const char *msg, int len);

void print_debug(const char *msg);

void print_debug(char c);

void send_to_pi(unsigned char *buf);

void sendAcknowledge(unsigned char *buf);

#endif