#include "hemo_funcs.h"

void trim_float(float *val, float min_f, float max_f) {
  if(*val < min_f) {
    *val = min_f;
  }
  else if(*val > max_f) {
    *val = max_f;
  }
}

/* Shifts a buffer's elements over and appends a new value to it */
void shift_into_buf(float *buf, int buf_size, float val) {
  int i;
  for(i = 0; i < buf_size - 1; i++) {
    buf[i] = buf[i+1];
  }
  buf[buf_size - 1] = val;
}

/* lazy copy of bzero */
void zero_buf(float *buf, int buf_size) {
	int i;
	for(i = 0; i < buf_size; i++) {
		buf[i] = 0;
	}
}

