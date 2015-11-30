#include <Lidar.h>

#define LIDAR_SERIAL Serial1

int scans_received = 0;

int deg = 0;

void setup() {
  Serial.begin(115200);
  LIDAR_SERIAL.begin(115200);

  init_lidar();

}

void loop() {
  loop_lidar();

  lidar_dist d = get_degree(deg++);
  if (deg > 350) { deg = 0; }
  
//  Serial.println(d);
  Serial.print (d.degree);
  Serial.print(" ");
  Serial.print(d.dist);
    Serial.print(" ");
  Serial.print(d.dist_x);
    Serial.print(" ");
  Serial.println(d.dist_y);

  if(flag_scan_complete) {  
    scans_received++;    
    while(Serial1.available() > 100) Serial1.read();
    reset_scan_buf();
  }
}
