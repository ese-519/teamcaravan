//#include <HMotor.h>
#include <motor_controller_comms.h>
#include <SerialMessage.h>
#include <Lidar.h>



unsigned char msg_buf[PACKET_SIZE];

#define LIDAR_SERIAL Serial1
#define RPI_SERIAL Serial2
#define MOTOR_SERIAL Serial3

void setup() {
  Serial.begin(115200);
  LIDAR_SERIAL.begin(115200);
  RPI_SERIAL.begin(115200);
  MOTOR_SERIAL.begin(115200);
  
  init_lidar();
}

void loop() {
  loop_lidar();
  
  for (int i = 0; i < 360; i++) {
     lidar_dist d = get_degree(i); 
  }

  if (RPI_SERIAL.available() >= PACKET_SIZE) {
    if (read_in_packet_2(msg_buf)) {
      Serial.println("Reading");
      processPacket();
    }
    else {
      Serial.println("Packet not read");
    }
  }
  
  int idx = 0;
  while (Serial.available()) {
    msg_buf[idx++] = Serial.read();
  }
  
  if (idx > 0) {
     int i = s_to_i(msg_buf, idx);
     lidar_dist d = get_degree(i%360);
     
     print_debug("Degree: ");
     for (int i = 0; i<idx; i++) {
       print_debug(msg_buf[i]);
     }
     print_debug(" ");
//     print_debug();
     print_debug("\r\n");
//     Serial.println(i_to_s((int)d.dist));
     
     Serial.print(i%360);
     Serial.print(" ");
     Serial.print (d.degree);
     Serial.print(" ");
     Serial.println(d.dist);
  }
//    char c = Serial.read();
//    Serial.print(c);
//    print_debug(c);
//  }
}



void processPacket() {
  int msg_type = c_to_i(msg_buf[1]);
  switch (msg_type) {
     case TURN_MSG:
       Serial.println("Turn");
       break;
  }
}
