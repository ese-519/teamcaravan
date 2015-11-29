//#include <HMotor.h>
//#include <motor_define.h>
#include <motor_controller_comms.h>

unsigned char buf[PACKET_SIZE];

#define RPI_SERIAL Serial2
#define MOTOR_SERIAL Serial3

long turn_time = 2000;
long turn_dur = 4000;

long next_turn = turn_time;
long done_turn = turn_time + turn_dur;
bool didTurn = false;

void setup() {
  Serial.begin(115200);
  MOTOR_SERIAL.begin(115200);
  RPI_SERIAL.begin(115200);
}

void loop() {
    
//    if (millis() > next_turn && millis() < done_turn) {
//      if (!didTurn) {
//           make_packet_vels(buf, 0.0, 0.0);
//           send_packet_serial_2(buf);   
//           delay(800);
//       } else {
//           make_packet_vels(buf, 22.0, -22.0);
//           send_packet_serial_2(buf);   
//       }
//       Serial.println("Turning");
//       
//       didTurn = true;
//    } else {
//      if (didTurn) {
//         next_turn = millis() + turn_time;
//         done_turn = next_turn + turn_dur;
//         didTurn = false;
//      }
//       make_packet_vels(buf, -7.0, -7.0);
//       send_packet_serial_2(buf);   
//       Serial.println("Straight");
//    }
  
  
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
    //send_packet_serial_2(buf);
}
