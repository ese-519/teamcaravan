/**
 * Example code for using a microchip mrf24j40 module to send and receive
 * packets using plain 802.15.4
 * Requirements: 3 pins for spi, 3 pins for reset, chip select and interrupt
 * notifications
 * This example file is considered to be in the public domain
 * Originally written by Karl Palsson, karlp@tweak.net.au, March 2011
 */
#include <SPImr.h>
#include <mrf24j.h>

#define PACKET_SIZE 10
#define PACKET_START 2  // STX
#define PACKET_END 3     // ETX

#define MOTOR_FWD   1
#define MOTOR_BWD   2
#define MOTOR_BRK   0


const int pin_reset = 6;
const int pin_cs = 10; // default CS pin on ATmega8/168/328 is 10
const int pin_int = 2; // default interrupt pin on ATmega8/168/328 is 2

Mrf24j mrf(pin_reset, pin_cs, pin_int);
#define PACKET_SIZE 10
char buf[PACKET_SIZE];

long last_time;
long tx_interval = 1000;

#define MY_ADDR 0x4001
#define THEIR_ADDR 0x5000

float vel_l = 0.0;
float vel_r = 0.0;

void setup() {
  Serial.begin(9600);
  
  mrf.reset();
  mrf.init();
  
  mrf.set_pan(0xcafe);
  // This is _our_ address
  mrf.address16_write(MY_ADDR); 

  // uncomment if you want to receive any packet on this channel
  mrf.set_promiscuous(true);
  
  // uncomment if you want to enable PA/LNA external control
  //mrf.set_palna(true);
  
  // uncomment if you want to buffer all PHY Payload
  //mrf.set_bufferPHY(true);

  attachInterrupt(digitalPinToInterrupt(pin_int), interrupt_routine, CHANGE); // interrupt 0 equivalent to pin 2(INT0) on ATmega8/168/328
  last_time = millis();
  interrupts();
}

void interrupt_routine() {
    //Serial.print("Int\n");
    mrf.interrupt_handler(); // mrf24 object interrupt routine
}

void loop() {
  
    int in = 0;
    if (Serial.available() > 0)
    {
        in = Serial.read();
        if (in == 119)
        {
            vel_l = vel_l == 0.0 ? 7.1 : 0.0;
            vel_r = vel_r == 0.0 ? 7.1 : 0.0;
        }
        else if (in == 97)
        {
            vel_l = -7.1;
            vel_r = 7.1;    
        }
        else if (in == 100)
        {
            vel_l = 7.1;
            vel_r = -7.1;    
        }
        else if (in == 115)
        {
            vel_l = vel_l == 0.0 ? -7.1 : 0.0;
            vel_r = vel_r == 0.0 ? -7.1 : 0.0;
        }
    }
    
    make_packet_vels(buf, vel_l, vel_r);
    send_packet_wireless();  
  
    mrf.check_flags(&handle_rx, &handle_tx);
    delay(10);
//    unsigned long current_time = millis();
//    if (current_time - last_time > tx_interval) {
//        last_time = current_time;
//        //mrf.send16(THEIR_ADDR, "abcd");
//    }
}

void send_packet_wireless() {
  char temp[PACKET_SIZE+1];
  int i;
  
  for(i = 0; i < PACKET_SIZE; i++) {
    temp[i] = (char)buf[i] + '0';
    buf[i] = 0;
  }
  temp[PACKET_SIZE] = '/0';
  mrf.send16(THEIR_ADDR, temp);
  
 for(i = 0; i < PACKET_SIZE+1; i++) {
    Serial.print((int)temp[i]);
    Serial.print('|');
  }
  Serial.println("");
}

void make_packet_vels(char *buf, float vel_l, float vel_r) {
  buf[0] = PACKET_START;
  buf[PACKET_SIZE-1] = PACKET_END;

  float spd_l = abs(vel_l);
  float spd_r = abs(vel_r);
  if(spd_l > .02) {
    buf[1] = vel_l > 0.0 ? MOTOR_FWD : MOTOR_BWD;
    buf[2] = (int) spd_l;
    buf[3] = (int)((spd_l - buf[2]) * 100);
  }
  if(spd_r > .02) {
    buf[4] = vel_r > 0.0 ? MOTOR_FWD : MOTOR_BWD;
    buf[5] = (int)spd_r;
    buf[6] = (int)((spd_r - buf[5]) * 100);
  }
  else {
    buf[1] = buf[2] = buf[3] = buf[4] = buf[5] = buf[6] = 0;
    buf[7] = buf[8] = 1;
  }
  //Serial.println("ret");
}

void handle_rx() {
    //Serial.print("received a packet ");Serial.print(mrf.get_rxinfo()->frame_length, DEC);Serial.println(" bytes long");
    
    if(mrf.get_bufferPHY()){
      //Serial.println("Packet data (PHY Payload):");
      for (int i = 0; i < mrf.get_rxinfo()->frame_length; i++) {
          //Serial.print(mrf.get_rxbuf()[i]);
      }
    }
    
    //Serial.println("\r\nASCII data (relevant data):");
    for (int i = 0; i < mrf.rx_datalength(); i++) {
        //Serial.write(mrf.get_rxinfo()->rx_data[i]);
    }
    
//    Serial.print("\r\nLQI/RSSI=");
//    Serial.print(mrf.get_rxinfo()->lqi, DEC);
//    Serial.print("/");
//    Serial.println(mrf.get_rxinfo()->rssi, DEC);
}

void handle_tx() {
    if (mrf.get_txinfo()->tx_ok) {
//        Serial.println("TX went ok, got ack");
    } else {
//        Serial.print("TX failed after ");Serial.print(mrf.get_txinfo()->retries);Serial.println(" retries\n");
    }
}
