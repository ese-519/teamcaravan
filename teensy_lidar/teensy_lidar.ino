#include <Lidar.h>

#define LIDAR_SERIAL Serial1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  LIDAR_SERIAL.begin(115200);

  init_lidar();

}

void loop() {
  loop_lidar();

  lidar_dist d = get_degree(0);
  
//  Serial.println(d);
  Serial.println(d.degree);
  Serial.println(d.dist);
  Serial.println(d.dist_x);
  Serial.println(d.dist_y);

}
