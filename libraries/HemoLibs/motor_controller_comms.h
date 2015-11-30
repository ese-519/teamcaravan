#include "Arduino.h"
#include "stdint.h"

#define PACKET_SIZE 10
#define PACKET_START 2   // STX
#define PACKET_END 3     // ETX

#define SERIAL_PORT Serial3
#define SERIAL_PORT_2 Serial2

int read_in_packet_2(unsigned char *buf);

int read_in_packet(unsigned char *buf);

void make_packet_vels(unsigned char *buf, float vel_l, float vel_r);

/* Todo: void make_packet_all(...) */

void send_packet_serial_2(unsigned char *buf);

void send_packet_serial(unsigned char *buf);
