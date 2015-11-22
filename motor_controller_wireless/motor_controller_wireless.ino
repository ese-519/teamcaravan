#include <HMotor.h>
#include <motor_define.h>
#include <motor_controller_comms.h>
#include <mrf24j.h>
#include <SPImr.h>

#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN  13
#define CS_PIN   10
#define INT_PIN  17
#define WAKE_PIN 16
#define RST_PIN  9

#define MY_ADDR 0x5000
#define THEIR_ADDR 0x4001

Mrf24j mrf(RST_PIN, CS_PIN, INT_PIN);

char buf[PACKET_SIZE];

HMotor motor_left(MOTOR_LEFT);
HMotor motor_right(MOTOR_RIGHT);

int received = 0;

long last_time;
long tx_interval = 1000;

long time_since_received_ms;

void mrf_isr() {
//  Serial.printf("Int\n");
  mrf.interrupt_handler();
}

long current_time;

void handle_rx() {
  Serial.print("Got something...\n");
  Serial.println(mrf.get_rxinfo()->frame_length);
    time_since_received_ms = millis();
    if(true) {// mrf.get_rxinfo()->frame_length == 18) {
      int i;
      unsigned char *rx_buf = mrf.get_rxinfo()->rx_data;
    for (int i = 0; i < mrf.rx_datalength(); i++) {
        Serial.write(mrf.get_rxinfo()->rx_data[i]);
        Serial.println();
    }
      for(i = 0; i < PACKET_SIZE; i++) {
        buf[i] = rx_buf[i] - '0';
       Serial.print((int)buf[i]);
        Serial.print('|');
      }
      Serial.println();
      //Serial.println(mrf.get_rxinfo()->frame_length);
      acceptCommands();
    }
    else {
      Serial.println("Brake");
      motor_left.setBrake();
      motor_right.setBrake();
    }
}



void handle_tx() {
    if (mrf.get_txinfo()->tx_ok) {
//        Serial.println("TX went ok, got ack");
    } else {
//        Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
    }
}



void setup() {
  Serial.begin(9600);
  
  mrf.reset();
  mrf.init();
  
  mrf.set_pan(0xcafe);
  mrf.address16_write(MY_ADDR);
  mrf.set_promiscuous(true);
  
  attachInterrupt(digitalPinToInterrupt(INT_PIN), mrf_isr, CHANGE);
  interrupts();
  
  motor_left.initMotor();
  motor_left.setBrake();

  motor_right.initMotor();
  motor_right.setBrake();
}

float vel;
long time;

void loop() {
  motor_left.updateMotor();
  motor_right.updateMotor();
  mrf.check_flags(&handle_rx, &handle_tx);
  if ((current_time = millis()) - time_since_received_ms > 200) {
    mrf.send16(THEIR_ADDR, "abcd");    
    //motor_left.setBrake();
    //motor_right.setBrake();
    //motor_right.setVelocity(2.0f);
  }
}



void acceptCommands() {
//  Serial.print("here\n");
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
        } else {
          motor_left.setBrake();  
        }
        vel = 0;
        vel += ((int)buf[2] % 23);
        vel += (float)((int)buf[3] % 100)/100;
        Serial.println(vel);
       //  Serial.print("\t");
        motor_left.setVelocity(vel);

        if(buf[4] == MOTOR_FWD) {
          motor_right.setDirection(MOTOR_FWD); 
        }
        else if(buf[4] == MOTOR_BWD) {
          motor_right.setDirection(MOTOR_BWD); 
        } else {
          motor_right.setBrake();  
        }
        vel = 0;
        vel += ((int)buf[5] % 23);
        vel += (float)((int)buf[6] % 100)/100; 
        motor_right.setVelocity(vel);
        Serial.println(vel);
       /* motor_left.updateMotor();
        motor_right.updateMotor();*/
      }
 }

