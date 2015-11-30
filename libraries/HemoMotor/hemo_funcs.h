#ifndef __HEMO_FUNCS_H__
#define __HEMO_FUNCS_H__

void trim_float(float *val, float min_f, float max_f);

/* Not a template to reduce overhead */
void shift_into_buf(float *buf, int buf_size, float val);

void zero_buf(float *buf, int buf_size);

#endif