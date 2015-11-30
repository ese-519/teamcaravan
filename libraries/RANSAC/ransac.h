#ifndef __RANSAC_H__
#define __RANSAC_H__


typedef struct equation_parameters {
	double m;
	double b;
	double avg_err;
} equation_parameters;

equation_parameters calculate_linear_regression(int degree1, int degree2, int iter);
	


#endif
