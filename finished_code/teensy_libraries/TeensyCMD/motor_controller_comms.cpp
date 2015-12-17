#include "motor_controller_comms.h"
#include "motor_define.h"
#include <Arduino.h>
#include "stdint.h"

#define SERIAL_PORT Serial3
#define SERIAL_PORT_2 Serial2

int c_to_i(int i) {
   return i - '0';
}

/* Basic read, checks for proper start byte */
int read_in_packet(unsigned char *buf) {
  int success = 0;
  int bytes_read = 0;
  uint8_t c;

  /* Find beginning of packet */
  while( SERIAL_PORT.available() && (c = SERIAL_PORT.read()) != PACKET_START) {
    Serial.println(c);  }
  buf[0] = PACKET_START;
  bytes_read++;

  /* Read rest of packet */
  while( ((c = SERIAL_PORT.read()) != -1) && bytes_read < PACKET_SIZE) {
    buf[bytes_read] = c;
    bytes_read++;
  }

  /* Verification */
  if(buf[PACKET_SIZE - 1] == PACKET_END) {
    success = 1;
  }

  return success;
}

/* Basic read, checks for proper start byte */
int read_in_packet_2(unsigned char *buf) {
  int success = 0;
  int bytes_read = 0;
  uint8_t c;

  /* Find beginning of packet */
  while( SERIAL_PORT_2.available() && (c = c_to_i(SERIAL_PORT_2.read())) != PACKET_START) {
    /*Serial.print(c);*/  }
  buf[0] = PACKET_START;
  bytes_read++;

  /* Read rest of packet */
  while( ((c = c_to_i(SERIAL_PORT_2.read())) != -1) && bytes_read < PACKET_SIZE) {
    buf[bytes_read] = c;
    bytes_read++;
  }

  /* Verification */
  if(buf[PACKET_SIZE - 1] == PACKET_END) {
    success = 1;
  }

  return success;
}


void make_packet_vels(unsigned char *buf, float vel_l, float vel_r) {
  buf[0] = PACKET_START;
  buf[PACKET_SIZE-1] = PACKET_END;

  float spd_l = abs(vel_l);
  float spd_r = abs(vel_r);
  if(spd_l > .02) {
    buf[1] = vel_l > 0.0 ? MOTOR_FWD : MOTOR_BWD;
    buf[2] = (int) spd_l;
    buf[3] = (int)((spd_l - buf[1]) * 100);
  }
  if(spd_r > .02) {
    buf[4] = vel_r > 0.0 ? MOTOR_FWD : MOTOR_BWD;
    buf[5] = (int) spd_r;
    buf[6] = (int)((spd_r - buf[4]) * 100);
  }
  else {
    buf[1] = buf[2] = buf[3] = buf[4] = buf[5] = buf[6] = 0;
  }
  //Serial.println("ret");
}

void send_packet_serial(unsigned char *buf) {
  int i;
  for(i =  0; i < PACKET_SIZE; i++) {
    Serial.print(buf[i]);
    SERIAL_PORT.write(buf[i]);
  }
}

void send_packet_serial_2(unsigned char *buf) {
  int i;
  for(i =  0; i < PACKET_SIZE; i++) {
    SERIAL_PORT_2.write(buf[i]);
  }
}