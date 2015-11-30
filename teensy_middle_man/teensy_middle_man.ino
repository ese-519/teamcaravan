//#include <HMotor.h>
#include <motor_controller_comms.h>
#include <SerialMessage.h>
#include <Lidar.h>

unsigned long lastTurn = 0;
unsigned long turnTimeout = 1000;
unsigned long lastMovePacket = 0;
unsigned long moveTimeout = 400;

unsigned char msg_buf[PACKET_SIZE];
unsigned char vel_buf[PACKET_SIZE];

#define LIDAR_SERIAL Serial1
#define RPI_SERIAL Serial2
#define MOTOR_SERIAL Serial3

#define F_START 150
#define F_END 230
#define L_START 279
#define L_END 281
#define R_START 99
#define R_END 101

// in mm
int hazardRange = 609;
int wallDiff = 609;

int turnSpeed = 4;
bool isStopped = true;

int distance(int deg) {
   lidar_dist d = get_degree(deg%360);
   return (int)d.dist;
}

float distanceInRange(int s, int e) {
   int cnt = 0;
   float res = 0;
   for (int i = s; i <= e; i++) {
      if (distance(i) != 0) {
         res += distance(i);
         cnt++; 
      }
   } 
   
   return res/cnt;
}

int scanFront() {
  int middle = (F_START+F_END)/2;
  
  int low;
  for (low = F_END; low >= middle; low--) {
      if (distance(low) != 0 && distance(low) <= hazardRange) {
        break;
      }
  }
  
  int high;
  for (high = F_START; high <= middle; high++) {
      if (distance(high) != 0 && distance(high) <= hazardRange) {
        break;
      }
  }
  
  if (low == high == middle) {
    // nothing found
    if (distance(middle) == 0 || distance(middle) >= hazardRange) {
      return 0;
    }
  }
  
  if (middle-low > high-middle) {
     // More object to right, so turn left (CW)
     return 1;
  } else if (middle-low < high-middle) {
     // More object to left, so turn right (CCW)
     return -1; 
  } else {
     return 0; 
  }
  
}

void setup() {
  Serial.begin(115200);
  LIDAR_SERIAL.begin(115200);
  RPI_SERIAL.begin(115200);
  MOTOR_SERIAL.begin(115200);
  
  init_lidar();
}

void brake() {
  Serial.println("Brake");
  isStopped = true;
  make_packet_vels(vel_buf, 0.0, 0.0);
  vel_buf[1] = i_to_c(0);
  send_packet_serial(vel_buf);
}

void loop() {
  
  loop_lidar();

//  if (millis() - lastMovePacket > moveTimeout) {
//     brake(); 
//  }
  
  if (RPI_SERIAL.available() >= PACKET_SIZE) {
//     while (RPI_SERIAL.available()) {
//        Serial.println(RPI_SERIAL.read()); 
//     }
//     Serial.println("");
    if (read_in_packet_2(msg_buf)) {
      lastMovePacket = millis();
      if (checkBrake()) {
         Serial.println("Braked"); 
      } else { 
        // Check front
        int frontObject = scanFront();
        if (frontObject == 0) { 
//          Serial.println("No obstacles");   

          // Don't worry about hallway centering if Pi wants us to turn
          if (!checkTurn()) {
            // Check left
            float leftWall = distanceInRange(L_START, L_END);
            // Check right
            float rightWall = distanceInRange(R_START, R_END);
            
            if (leftWall - rightWall > wallDiff) {
              //left wall far away -- turn left (CW) 
              turn(turnSpeed);
            } else if (rightWall - leftWall > wallDiff) {
              //right wall far away -- turn right (CCW) 
              turn(-turnSpeed);
            }
          }
          
          // No obstacles in front, adjusted for hallway centering, process Pi message:
          processPacket();
          
        } else {
          turn(turnSpeed * frontObject);
//          Serial.print("Oobstacles at ");   
//          Serial.println(frontObject);
        }
      }
    }
    else {
      Serial.println("Packet not read");
    }
  }
  
/* #END REGION */

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

float getVel(int off) {
  float vel = 10 * msg_buf[2 + off] + msg_buf[3 + off] + (msg_buf[4 + off] / 10.0);
  return vel;
}

void drive(float vel) {
  if (vel > 0) {
    Serial.print("Forward: ");
  } else {
    Serial.print("Backward: ");
  }
  Serial.println(vel);
  make_packet_vels(vel_buf, vel, vel);
  send_packet_serial(vel_buf);
}

void turn(float vel) {
  if (vel > 0) {
     Serial.print("Turning left: ");
     Serial.println(vel);
     make_packet_vels(vel_buf, -vel, vel);
     send_packet_serial(vel_buf);
   } else {
     Serial.print("Turning right: ");
     Serial.println(vel);
     make_packet_vels(vel_buf, vel, -vel);
     send_packet_serial(vel_buf);
   }
}

bool checkBrake() {
  if (msg_buf[1] == BRK_MSG) {
     brake(); 
     return true;
  }
  
  return false;
}

bool checkTurn() {
  if (msg_buf[1] == TURN_MSG) {
     return true;
  }
  
  return false;
}

void processPacket() {  
//  for (int i = 0; i < PACKET_SIZE; i++) {
//     Serial.println(msg_buf[i]);
//  } 

  int msg_type = msg_buf[1];
  float vel;
  int dir;
  switch (msg_type) {
     case TURN_MSG:
       vel = getVel(1);
       dir = c_to_i(msg_buf[2]);
       if (dir == DIR_LEFT) {
         turn(vel);
       } else if (dir == DIR_RIGHT) {
         turn(-vel);
       }
       break;
     case FWD_MSG:
       vel = getVel(0);
       drive(vel);
       break;
     case BWD_MSG:
       vel = getVel(0);
       drive(vel);
       break;
     case BRK_MSG:
       brake();
       break;
  }
}
