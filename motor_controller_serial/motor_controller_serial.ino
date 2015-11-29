#include <HMotor.h>
#include <motor_define.h>
#include <motor_controller_comms.h>

unsigned char buf[PACKET_SIZE];

HMotor motor_right(MOTOR_RIGHT);
HMotor motor_left(MOTOR_LEFT);

int received = 0;

long last_packet;

#define MOTOR_SERIAL Serial3

void setup() {
  Serial.begin(115200);
  MOTOR_SERIAL.begin(115200);
  
  motor_left.initMotor();
  motor_left.setBrake();

  motor_right.initMotor();
  motor_right.setBrake();
  digitalWrite(13, HIGH);
  Serial.print("Ready\r");
}
float vel;
long time;

void loop() {
  motor_left.updateMotor();
  motor_right.updateMotor();
  
//  vel = 0;
//  vel += ((int)22 % 23);
//  vel += (float)((int)1 % 100)/100;
//  
//  motor_right.setDirection(MOTOR_FWD); 
//  motor_left.setDirection(MOTOR_BWD); 
//  
//  motor_right.setVelocity(vel);
//  motor_left.setVelocity(vel);
  
 
  if(MOTOR_SERIAL.available() >= PACKET_SIZE) {
//    Serial.println(MOTOR_SERIAL.available());
//    Serial.println(PACKET_SIZE);
//    Serial.println(MOTOR_SERIAL.read());
    if(read_in_packet(buf)) {
      Serial.print("Reading\r");
      acceptCommands();
      last_packet = millis();
    }
    else if (millis() - last_packet > 250) {
      Serial.print("Braking\r");
      motor_left.setBrake();
      motor_right.setBrake();
    }
  }
}

void acceptCommands() {
  
   for (int i = 0; i < PACKET_SIZE; i++) {
      Serial.print(buf[i]); 
   }
   Serial.println();
  
   if(buf[1] == 0) {
     motor_left.setBrake();
     motor_right.setBrake();
   }
   else {
     if(buf[1] == MOTOR_FWD) {
       motor_left.setDirection(MOTOR_FWD);
       /*Serial.println("ey"); */        }
        else if(buf[1] == MOTOR_BWD) {
          motor_left.setDirection(MOTOR_BWD); 
        }
        vel = 0;
        vel += ((int)buf[2] % 23);
        vel += (float)((int)buf[3] % 100)/100;
       // Serial.print(vel);
        //Serial.print('\t');
        motor_left.setVelocity(vel);

        if(buf[4] == MOTOR_FWD) {
          motor_right.setDirection(MOTOR_FWD); 
        }
        else if(buf[4] == MOTOR_BWD) {
          motor_right.setDirection(MOTOR_BWD); 
        }
        vel = 0;
        vel += ((int)buf[5] % 23);
        vel += (float)((int)buf[6] % 100)/100; 
      //  Serial.println(vel);
        motor_right.setVelocity(vel);
        Serial.println();
        /*Serial.println(vel);*/
       /* motor_left.updateMotor();
        motor_right.updateMotor();*/
      }
 }

