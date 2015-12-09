#include <Lidar.h>
// #include <math.h>

int lidar_idx = 0;
lidar_dist lidar_dist_buf_complete[360];
uint8_t lidar_buf[BYTES_PER_PACKET];

// To main
void init_lidar() {
	lidar_idx = 0;
}

lidar_dist get_degree(int des_degree) {
	return lidar_dist_buf_complete[des_degree%360];
}

int process_packet() {

	int buf_idx = 0;

	if(lidar_buf[1] < 0xA0 || lidar_buf[1] > 0xF9) {
		// Serial.println("Not in range");
		return 1;
	}

	for(int k = 0; k < 4; k++) {
		int temp_ind = 5+k*4;
		// "invalid data" flag
		if(lidar_buf[temp_ind] & 0x80) {
			int index = (4*(lidar_buf[1] - 160)) + k;
	
			uint16_t deg = (4*(lidar_buf[1] - 160)) + k;
			lidar_dist_buf_complete[index].degree = deg;
			lidar_dist_buf_complete[index].dist = 0;
			lidar_dist_buf_complete[index].dist_x = 0; //(int16_t)(cur_dist * cos((double)PI/180 * (double)deg));
			lidar_dist_buf_complete[index].dist_y = 0; //(int16_t)(cur_dist * sin((double)PI/180 * (double)deg));

			// Serial.print("Invalid ");
			// Serial.println(index);
			// Serial.print(temp_ind);
			// Serial.print(" ");
			// for (int i = 0; i < BYTES_PER_PACKET; i++) {
			// 	Serial.print(lidar_buf[i], HEX);
			// 	Serial.print(" ");
			// }
			// Serial.println("");
		} else {
			int index = (4*(lidar_buf[1] - 160)) + k;
			uint16_t deg = (4*(lidar_buf[1] - 160)) + k;
			uint16_t cur_dist = ((lidar_buf[5 + 4*k] & 0x3F) << 8) + lidar_buf[4+4*k];

			// Serial.print("Storing ");
			// Serial.println(index);

			lidar_dist_buf_complete[index].degree = deg;
			lidar_dist_buf_complete[index].dist = cur_dist;
			lidar_dist_buf_complete[index].dist_x = (int16_t)(cur_dist * cos((double)PI/180 * (double)deg));
			lidar_dist_buf_complete[index].dist_y = (int16_t)(cur_dist * sin((double)PI/180 * (double)deg));
		}
	} 

	return 0; 
}

// To main
void loop_lidar() {

  while (Serial1.available() && lidar_idx < BYTES_PER_PACKET) {

  	// Skip to beginning of packet
  	if (Serial1.peek() != 0xFA) {
  		Serial1.read();
  	} else {
  		// Found beginning
  		// Serial.print("Reset... ");
  		// Serial.println(lidar_idx);
  		lidar_idx = 0;
  		lidar_buf[lidar_idx++] = Serial1.read();

  		// Copy either entire packet or until we find the start of the next packet
  		while (lidar_idx < BYTES_PER_PACKET && Serial1.peek() != 0xFA) {
  			if (Serial1.available()) {
	  			// Serial.print(Serial1.peek(), HEX);
	  			// Serial.print(" ");
	  			lidar_buf[lidar_idx++] = Serial1.read();
	  		}
  		}
  	}
  }

  // we read an entire packet
  if (lidar_idx == BYTES_PER_PACKET) {
  	// Serial.println("Processing...");
  	process_packet();
  	lidar_idx = 0;
  }
}
