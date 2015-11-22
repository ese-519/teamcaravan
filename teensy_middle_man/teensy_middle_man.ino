#include <HMotor.h>
#include <motor_define.h>
#include <motor_controller_comms.h>

unsigned char buf[PACKET_SIZE];
#define MOTOR_SERIAL Serial3

void setup() {
  Serial.begin(115200);
  MOTOR_SERIAL.begin(115200);
}

void loop() {
  if (Serial.available()) {
      char c = Serial.read();
      if (c == 'w') {
        make_packet_vels(buf, 2.0, 2.0);
        send_packet_serial(buf);
        Serial.print("Here");
      }
  }
}
