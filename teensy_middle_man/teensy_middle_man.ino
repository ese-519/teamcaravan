#include <HMotor.h>
#include <motor_define.h>
#include <motor_controller_comms.h>

unsigned char buf[PACKET_SIZE];

#define RPI_SERIAL Serial2
#define MOTOR_SERIAL Serial3

void setup() {
  Serial.begin(115200);
  MOTOR_SERIAL.begin(115200);
  RPI_SERIAL.begin(115200);
}

void loop() {
  if (RPI_SERIAL.available() >= PACKET_SIZE) {
    if(read_in_packet_2(buf)) {
      Serial.println("Reading");
      forwardCommands();
    }
    else {
      Serial.println("Packet not read");
    }
  }
}

void forwardCommands() {
    send_packet_serial_2(buf);
}
