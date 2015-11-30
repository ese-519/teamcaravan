#include <HMotor.h>
#include <motor_define.h>
#include <mrf24j.h>

#define HWSERIAL Serial3

#define PACKET_SIZE 10
#define PACKET_START 2  // STX
#define PACKET_END 3     // ETX

char buf[PACKET_SIZE];

#define MOSI_PIN 11
#define MISO_PIN 12
#define SCK_PIN  13
#define CS_PIN   10
#define INT_PIN  15
#define WAKE_PIN 16
#define RST_PIN  14

#define MY_ADDR 0x5000
#define THEIR_ADDR 0x4001
#define WIRELESS_CHANNEL 0x14

HMotor motor_left(MOTOR_LEFT);
HMotor motor_right(MOTOR_RIGHT);

Mrf24j mrf(RST_PIN, CS_PIN, INT_PIN);

int received = 0;

long last_time;
long tx_interval = 1000;


void mrf_isr() {
  mrf.interrupt_handler();
}

void handle_rx() {
    
    if(mrf.get_rxinfo()->frame_length == 23) {
      int i;
      unsigned char *rx_buf = mrf.get_rxinfo()->rx_data;
      for(i = 0; i < PACKET_SIZE; i++) {
        buf[i] = rx_buf[i];
      }
      acceptCommands();
      Serial.println("k");
    }
    else {
      //Serial.println("Brake");
      motor_left.setBrake();
      motor_right.setBrake();
    }
}


void handle_tx() {
    if (mrf.get_txinfo()->tx_ok) {
        Serial.println("TX went ok, got ack");
    } else {
        Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
    }
}


/* Basic read, checks for proper start byte */
int readInPacket(char *buf) {
  int success = 0;
  int bytes_read = 0;
  char c;

  /* Find beginning of packet */
  while( HWSERIAL.available() && (c = HWSERIAL.read()) != PACKET_START) {
    /*Serial.print(c);*/  }
  buf[0] = PACKET_START;
  bytes_read++;

  /* Read rest of packet */
  while( ((c = HWSERIAL.read()) != -1) && bytes_read < PACKET_SIZE) {
    buf[bytes_read] = c;
    bytes_read++;
  }

  /* Verification */
  if(buf[PACKET_SIZE - 1] == PACKET_END) {
    success = 1;
  }

  return success;
}

void setup() {
  Serial.begin(115200);
  HWSERIAL.begin(115200);
  
  pinMode(CS_PIN, OUTPUT);
  pinMode(WAKE_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  pinMode(INT_PIN, INPUT);
  
  mrf.reset();
  mrf.init();
  
  mrf.set_channel(WIRELESS_CHANNEL);
  mrf.set_pan(0xcafe);
  mrf.address16_write(MY_ADDR);
  
  attachInterrupt(INT_PIN, mrf_isr, CHANGE);
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
  unsigned long current_time = millis();
    if (current_time - last_time > 1000) {
        last_time = current_time;
        Serial.println("txxxxxxxxxxing...");
        mrf.send16(THEIR_ADDR, "abcd");
    }

}

void testOfTime() {
  // time = micros();
  motor_right.updateMotor();
  motor_left.updateMotor();
 
 acceptCommands();
  Serial.print(motor_left._tgt_vel);
  Serial.print("\t");
  Serial.print(motor_left._cur_vel);
  Serial.print("\t");
  Serial.print(motor_right._tgt_vel);
  Serial.print("\t");
  Serial.println(motor_right._cur_vel);
}

void acceptCommands() {
   if(buf[1] == 0) {
     motor_left.setBrake();
     motor_right.setBrake();
   }
   else {
     if(buf[1] == 1) {
       motor_left.setDirection(MOTOR_FWD);
       /*Serial.println("ey"); */        }
        else if(buf[1] == 2) {
          motor_left.setDirection(MOTOR_BWD); 
        }
        vel = 0;
        vel += ((int)buf[2] % 23);
        vel += (float)((int)buf[3] % 100)/100;
        /*Serial.print(vel);
         Serial.print("\t");*/
        motor_left.setVelocity(vel);

        if(buf[4] == 1) {
          motor_right.setDirection(MOTOR_FWD); 
        }
        else if(buf[4] == 2) {
          motor_right.setDirection(MOTOR_BWD); 
        }
        vel = 0;
        vel += ((int)buf[5] % 23);
        vel += (float)((int)buf[6] % 100)/100; 
        motor_right.setVelocity(vel);
        /*Serial.println(vel);*/
       /* motor_left.updateMotor();
        motor_right.updateMotor();*/
      }
 }

