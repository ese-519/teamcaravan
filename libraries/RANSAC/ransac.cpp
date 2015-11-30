#include "ransac.h"
#include <Lidar.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <math.h> 

equation_parameters temp;
equation_parameters best;


equation_parameters calculate_linear_regression(int degree1, int degree2, int iter){
int i = 0;
int nonzero_count = 0;
best.m = 0;
best.b = 0;
best.avg_err = 100000000;
	for(;i < iter;i++){
		
//		Serial.print("i: ");
//		Serial.println(i);
		
//		Serial.println("Creating random points");
//		int a = get_degree(rand()%(degree2-degree1 + 1) + degree1).dist_x;
//		int b = get_degree(rand()%(degree2-degree1 + 1) + degree1).dist_x;
//		Serial.print(a);
//		Serial.print("\t");
//		Serial.println(b);

		lidar_dist rd1 = get_degree(rand()%(degree2-degree1 + 1) + degree1);
		lidar_dist rd2 = get_degree(rand()%(degree2-degree1 + 1) + degree1);
//		Serial.println("Random Points Created");
		
		
		if(rd2.dist_x  - rd1.dist_x == 0){
			continue;
		}else {		
			temp.m = ((double)rd2.dist_y - (double)rd1.dist_y )/((double)rd2.dist_x  - (double)rd1.dist_x );
		}
//		Serial.println("Calculating B");
		temp.b =  rd1.dist_y -temp.m*rd1.dist_x;
		
//		Serial.println(2);
		int j = degree1;
		for(;j <= degree2; j++){
//		Serial.println(3);
			lidar_dist dp = get_degree(j);
			if(dp.dist_y || dp.dist_x){
//			Serial.println(4);
				temp.avg_err += abs((rd2.dist_y   - rd1.dist_y) * dp.dist_x - 
							(rd2.dist_x  - rd1.dist_x) * dp.dist_y +
							rd2.dist_x  * rd1.dist_y - rd2.dist_y  * rd1.dist_x)/
							(sqrt(pow(rd2.dist_y   - rd1.dist_y,2) + 
							 pow(rd2.dist_x   - rd1.dist_x,2)));
				nonzero_count++;
//				Serial.println(5);
			}
		}
		temp.avg_err /= nonzero_count;
//		Serial.println(6);
		if(temp.avg_err < best.avg_err){
//		Serial.println(7);
			best.m = temp.m;
			best.b = temp.b;
			best.avg_err = temp.avg_err;
			
		} 
	}
	
	return best;
}