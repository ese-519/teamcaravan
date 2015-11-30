// #include <motor_define.h>
#include "../TeensyCMD/motor_controller_comms.h"
#include "SerialMessage.h"
#include <math.h>

bool is_numeric(unsigned char c) {
	return c >= 48 && c <= 57;
}

int c_to_i(unsigned char c) {
	return c - '0';
}

unsigned char i_to_c(int i) {
	return i + '0';
}

char* i_to_s(int i) {
	// char buf [4];
	// sprintf (buf, "%03i", i);
	// return buf;
	// char res[20];
	// int in = i;
	// int dec = 0;
	// while (in > 0) {
	// 	in /= 10;
	// 	dec++;
	// }

	// Serial.print(dec);

	// int limit = dec;
	// for (int j = 0; j <= limit; j++) {
	// 	res[j] = i/pow(10, --dec);
	// 	Serial.print(i/pow(10, dec));
	// }

	// res[limit+1] = '\0';
	// return res;
}

int s_to_i(unsigned char* s, int l) {
	int res = 0;
	for (int i = 0; i < l; i++) {
		if (is_numeric(s[i])) {
			if (res != 0) {
				res *= 10;
			}
			res += c_to_i(s[i]);
		}
		else {
			return res;
		}
	}

	return res;
}

void make_packet(unsigned char *buf, const unsigned char *msg, int len) {
  buf[0] = PACKET_START;
  buf[PACKET_SIZE-1] = PACKET_END;

  buf[1] = INFO_MSG;

  for (int i = 2; i < len && i < PACKET_SIZE-1; i++) {
  	buf[i] = msg[i-2];
  }  
  
}

void print_debug(const char *msg) {
	// for (int i = 0; i < len; i++) {
		RPI_SERIAL.print(msg);
	// }
	// RPI_SERIAL.print("\r\n");
	// make_packet(buf, msg, len);
	// buf[1] = DEBUG_MSG;
	// send_to_pi(buf);
}

void print_debug(char c) {
	RPI_SERIAL.print(c);
}