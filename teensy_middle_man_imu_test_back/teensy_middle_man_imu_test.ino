
/*

Changed Brake()
changed send_ack();
changed headingCorrection
changed newTurn

*/

int driveDist = 75;
int rotDist = 90;

//#include <HMotor.h>
#include <motor_controller_comms.h>
#include <SerialMessage.h>
#include <Lidar.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_9DOF                dof   = Adafruit_9DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

float headingCorrection;
sensors_vec_t   orientation;

float heading = 0;

float v_0, v_1 = 0;
float x_0, x_1 = 0;
float t_0, t_1, t_d = 0;
float totalX = 0;

float last_a = 0;

bool waitingNewPacket = false;

bool go = false;

char inChar = 'b';
char lastChar = 'b';

unsigned char last_msg_buf[PACKET_SIZE];

float target_x = 0;
float target_deg = 0;

float drive_vel = 3.9;
float turn_vel = 3.9;

int turnLength = 1000;

bool checkObstacles = true;
bool checkHallway = false;
bool newTurn = false;

bool isDoingTurn = false;
bool isDoingDrive = false;

bool isCenter = true;
bool isLeft = false;
bool isRight = false;
unsigned long lastTurn = 0;
unsigned long turnTimeout = 1000;
unsigned long lastMovePacket = 0;
unsigned long moveTimeout = 400;

unsigned long lastSerPacket = 0;

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
int hazardRange = 609; // 2FT
int wallDiff = 304; // 1FT
int wallIgnoreRange = 3048; //10FT

int turnSpeed = 4;
bool isStopped = true;

void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  
  gyro.enableAutoRange(true);
  
  /* Initialise the sensor */
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
}

int ftToMm(float ft) {
   return (int)ft * 304.8; 
}

int distance(int deg, bool z = true) {
   lidar_dist d = get_degree(deg%360);
   
   if ((int)d.dist == 0 && z) {
      return distanceInRange(deg-1, deg+1); 
   }
   
   return (int)d.dist;
}

float distanceInRange(int s, int e) {
   int cnt = 0;
   float res = 0;
   for (int i = s; i <= e; i++) {
      if (distance(i, false) != 0) {
         res += distance(i, false);
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
  
  if (low == high && high == middle) {
    // nothing found
    if (distance(middle) == 0 || distance(middle) >= hazardRange) {
      return 0;
    }
  }
  
  if (low-middle > middle-high) {
     // More object to left, so turn right (CW)
     return -1;
  } else if (low-middle < middle-high) {
     // More object to right, so turn left (CCW)
     return 1;
  } else {
     return 0; 
  }
  
}

float getVi(float deltaT, float a) {
    return v_0 + a * deltaT;
}

float getXi(float deltaT, float a) {
    return x_0 + (v_0 * deltaT) + (0.5 * a * deltaT * deltaT);
}

void processImu() {
    sensors_event_t event; 
    sensors_event_t mag_event;   
    sensors_event_t event_gyro; 
    
    accel.getEvent(&event);
    mag.getEvent(&mag_event);
    gyro.getEvent(&event_gyro);
    dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation);
   
    float a = abs(event.acceleration.x);
  
    t_0 = t_1;
    t_1 = millis();
    t_d = (t_1 - t_0)/1000;
    
    float rad_s = event_gyro.gyro.z;
    
    if (abs(rad_s) <= 0.016) { 
        rad_s = 0;
    }
    
    float deg = (rad_s * t_d) * 180 / PI;
    heading = scaleHeading(heading + deg);
    
//    Serial.print("A: "); Serial.print(a); Serial.print(" Deg: "); Serial.print(deg); Serial.print(" Rad_s: "); Serial.println(rad_s);
        
    if (abs(a) < 0.3) { 
        a = 0;
    }
    
    last_a = a;

    float last_v = v_1;
    v_1 = getVi(t_d, a);
    
    float last_x = x_1;
    x_1 = getXi(t_d, a);
    
    v_0 = last_v;
    x_0 = last_x;
    
    totalX += x_1-x_0;
    
//    Serial.print("Total: "); Serial.print(totalX);
//    Serial.print(" A: "); Serial.print(a);
//    Serial.print(" V: "); Serial.println(v_1);
}

void setup() {
  Serial.begin(115200);
  LIDAR_SERIAL.begin(115200);
  RPI_SERIAL.begin(115200);
  MOTOR_SERIAL.begin(115200);
  
  initSensors();
  
  // Process IMU to get initial heading correction (to zero)
  processImu();
  headingCorrection = -1 * orientation.heading;
  
  init_lidar();
}

void brake() {
//  Serial.println("Brake");
  isStopped = true;
  make_packet_vels(vel_buf, 0.0, 0.0);
  vel_buf[1] = 0;
  send_packet_serial(vel_buf);
  
  v_0, v_1 = 0;
  x_0, x_1 = 0;
}

void print(const char *c) {
  Serial.print(c);
//  print_debug(c);
}

void println(const char *c) {
  Serial.println(c);
//  print_debug(c);
//  print_debug('\n'); 
}

void sendAck() {
//    Serial.print("Total: "); Serial.print(totalX);
//    Serial.print("Target: "); Serial.print(target_x);
//    Serial.print(" A: "); Serial.print(last_a);
//    Serial.print(" V: "); Serial.println(v_1); 
//    
//    Serial.print("Target Deg: "); Serial.print(getTargDeg()); 
//    Serial.print(" Cur Deg: "); Serial.println(getOrient()); 
//    
//    Serial.println("!!!!ACK!!!!");
 
    waitingNewPacket = true;
    RPI_SERIAL.print(ACK_MSG);
    delay(turnLength);
}

bool checkNewPacket() {
  
//    if (lastChar != inChar)
//      Serial.print(lastChar); Serial.print(" : "); Serial.println(inChar);
  
    return inChar != lastChar;
  
//   if (msg_buf[1] != TURN_MSG && msg_buf[1] != FWD_MSG) {
//       return false;
//   }
//     
//   for (int i = 0; i < PACKET_SIZE; i++) {
//       if (last_msg_buf[i] != msg_buf[i]) {
//           return true;
//       }
//   }
//   
//   return false;
}

float scaleHeading(float in) {
  while (in >= 360) {
     in = in - 360; 
  }
  
  while (in < 0) {
     in = in + 360; 
  }
  
  return in;
}

float getTargDeg() {
   return scaleHeading(target_deg); 
}

float getOrient() { 
   //return scaleHeading(orientation.heading + headingCorrection);
   return heading;
}

void loop() {
  
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'd' || c == 't' || c == 'b') {
      inChar = c;
      Serial.println(inChar);
    }
  }
  
  loop_lidar();

  processImu();

//  if (millis() - lastSerPacket > 500) {
//      lastSerPacket = millis();
//      char x_buf[10];
//      String(totalX).toCharArray(x_buf, 10);
////      println(x_buf);
//  }

  if (millis() - lastMovePacket > moveTimeout || waitingNewPacket) {
     brake(); 
  }
  
  if (totalX >= target_x && target_x != 0 && isDoingDrive) {
      isDoingDrive = false;
      sendAck();
      target_x = 0;
  }
  
  if (abs(getOrient() - getTargDeg()) < 5 && isDoingTurn) {
//     Serial.print("Updating Heading Cor.: "); Serial.println(headingCorrection);
     headingCorrection = -1 * orientation.heading;
     heading = getTargDeg();
//     Serial.print("Updating Heading Cor.: "); Serial.println(headingCorrection);
     isDoingTurn = false;
     sendAck();
  }
  
  if (true) {//RPI_SERIAL.available() >= PACKET_SIZE) {
//     while (RPI_SERIAL.available()) {
//        Serial.println(RPI_SERIAL.read()); 
//     }
//     Serial.println("");
    if (true) {
      lastMovePacket = millis();
      
      if (checkNewPacket()) {
         lastChar = inChar;
         Serial.println("new pkt");
         brake();
         delay(turnLength);
         if (inChar == 't') {
            isDoingTurn = true;
            newTurn = true;
            target_deg = scaleHeading(getOrient());
         }
         else if (inChar == 'd') {
            isDoingDrive = true; 
         }
         
         waitingNewPacket = false;
//         for (int i = 0; i < PACKET_SIZE; i++) {
//            last_msg_buf[i] = msg_buf[i];
//         }
      }
      
      if (checkBrake() || waitingNewPacket) {
//         Serial.println("Braked"); 
      } else { 
        checkInfo();
        
        // Check front
        int frontObject = scanFront();
        if (frontObject == 0 || !checkObstacles) { 
//          Serial.println("No obstacles");

          // Don't worry about hallway centering if Pi wants us to turn
          if (!checkTurn()) {
            // Check left
            float leftWall = distanceInRange(L_START, L_END);
            // Check right
            float rightWall = distanceInRange(R_START, R_END);
            
            if (checkHallway && leftWall - rightWall > wallDiff && leftWall < wallIgnoreRange && (isCenter || isRight)) {
              //left wall far away -- turn left (CW)
              println("Left wall far");
              isCenter = false;
              isLeft = true;
              isRight = false;
              brake();
              delay(turnLength);
              turnWithVel(turnSpeed);
              delay(turnLength);
              brake();
            } else if (checkHallway && rightWall - leftWall > wallDiff && rightWall < wallIgnoreRange && (isCenter || isLeft)) {
              //right wall far away -- turn right (CCW) 
              println("Right wall far");
              isCenter = false;
              isRight = true;
              isLeft = false;
              brake();
              delay(turnLength);
              turnWithVel(-turnSpeed);
              delay(turnLength);
              brake();
            } else {
               isCenter = true; 
               isLeft = false;
               isRight = false;
            }
          }
          
          // No obstacles in front, adjusted for hallway centering, process Pi message:
          processPacket();
          
        } else {
//          println("Obstacle");
//          Serial.print("Obstacles at ");   
//          Serial.println(frontObject);
          turnWithVel(turn_vel * frontObject);
        }
      }
    }
    else {
      Serial.println("Packet not read");
    }
  }
}

float getVel(int off) {
  float vel = 10 * msg_buf[2 + off] + msg_buf[3 + off] + (msg_buf[4 + off] / 10.0);
  return vel;
}

void drive(float dist) {
//  if (vel > 0) {
//    println("Forward: ");
//  } else {
//    println("Backward: ");
//  }
//  Serial.println(vel);
  if (target_x == 0) {
     totalX = 0; 
  }
  target_x = dist;
  make_packet_vels(vel_buf, drive_vel, drive_vel);
  send_packet_serial(vel_buf);
}

void turnWithVel(float vel) {
  if (vel > 0) {
//     print("Turning left: ");
//     Serial.println(vel);
     make_packet_vels(vel_buf, vel, -vel);
     send_packet_serial(vel_buf);
   } else {
//     print("Turning right: ");
//     Serial.println(vel);
     vel = -vel;
     make_packet_vels(vel_buf, -vel, vel);
     send_packet_serial(vel_buf);
   }
}

void turn(float deg) {
//  if (target_deg != deg) {
//  }
  
  if (newTurn) {
     newTurn = false;
     target_deg = getOrient() + deg; 
     Serial.print("New turn: "); Serial.print(deg); Serial.print(" Tar: "); Serial.println(target_deg);
  }
  
  Serial.print("Target Deg: "); Serial.print(getTargDeg()); 
  Serial.print(" Cur Deg: "); Serial.println(getOrient()); 
  
  make_packet_vels(vel_buf, turn_vel, -turn_vel);
  send_packet_serial(vel_buf);
}

bool checkBrake() {
  return inChar == 'b';
  if (msg_buf[1] == BRK_MSG) {
     brake(); 
     return true;
  }
  
  return false;
}

bool checkTurn() {
  return inChar == 'b';
//  if (msg_buf[1] == TURN_MSG) {
//     return true;
//  }
  
  return false;
}

void checkInfo() {
    false;
//   if (msg_buf[1] == INFO_MSG) {
//      processPacket();
//   } 
}

void processPacket() {  
//  for (int i = 0; i < PACKET_SIZE; i++) {
//     Serial.println(msg_buf[i]);
//  } 
     int vel; 
     switch (inChar) {
     case 't':
       vel = rotDist;
       turn(vel);
       break;
     case 'd':
       vel = driveDist;
       drive(vel);
       break;
     case 'b':
       brake();
       break;
     }
}
    
//  int msg_type = msg_buf[1];
//  float vel;
//  int dir;
//  switch (msg_type) {
//     case TURN_MSG:
//       vel = getVel(1);
//       dir = msg_buf[2];
//       // for now, always just turn the given deg ("vel")
//       turn(vel);
////       if (dir == DIR_LEFT) {
////         turn(vel);
////       } else if (dir == DIR_RIGHT) {
////         turn(-vel);
////       } else {
////         turn(vel); 
////       }
//       break;
//     case FWD_MSG:
//       vel = getVel(0);
//       drive(vel);
//       break;
//     case BWD_MSG:
//       vel = getVel(0);
//       drive(vel);
//       break;
//     case BRK_MSG:
//       brake();
//       break;
//     case INFO_MSG:
//       vel = getVel(1);
//       Serial.println("INFO");
//       Serial.println(vel);
//       int info_type = msg_buf[2];
//       switch (info_type) {
//          case INFO_OBS:
//            checkObstacles = !checkObstacles;
//            if (checkObstacles) {
//              println("Obs True"); 
//            } else {
//              println("Obs False"); 
//            }
//            break;
//          case INFO_HLWY:
//            checkHallway = !checkHallway;
//            if (checkHallway) {
//              println("Hallway True"); 
//            } else {
//              println("Hallway False"); 
//            }
//            break;
//          case INFO_HAZ_RNG:
//            hazardRange = ftToMm(vel);
//            break;
//          case INFO_WALL_RNG:
//            wallDiff = ftToMm(vel);
//            break;
//       }
//       break;
//  }
//}
