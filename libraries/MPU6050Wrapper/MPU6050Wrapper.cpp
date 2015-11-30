#include <MPU6050Wrapper.h>
#include <Arduino.h>
#include <Wire.h>
#include <Kalman.h>
#include <String.h>

/*Base Variables*/
//---------------------------------------------------------------------------------
MPU6050 imu;

double  accXd,accYd;
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
//---------------------------------------------------------------------------------


/*Kalman Filter Variables*/
//---------------------------------------------------------------------------------
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ;

uint32_t timer;

double gyroXangle, gyroYangle,gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY,compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY,kalAngleZ; // Calculated angle using a Kalman filter
//---------------------------------------------------------------------------------

/*Data Manipulation*/
//---------------------------------------------------------------------------------
imu_packet p0;
int sum_angle = 0;
int lastRotation = 0;
double currentAngle = 0;

double lastAccXd,lastAccYd;
double currentVXd,currentVYd;
double lastVXd,lastVYd;
double x_pos,y_pos;
long last_time = 0;
long current_time = 0;

//---------------------------------------------------------------------------------

/*Saving Data Variables*/
//---------------------------------------------------------------------------------
const char data_filename[] = "/usr/IMU_";
const char data_filename_end[] = ".csv";
FILE *file;
//---------------------------------------------------------------------------------


 imu_packet get_data(){
	imu_packet p;
	p.ax = accX;
	p.ay = accY;
	p.az = accZ;
	p.gx = gyroX;
	p.gy = gyroY;
	p.gz = gyroZ;
	return p;
 }

 int imu_to_degrees(){
	int rotation = get_data().gz-p0.gz;
	if(abs(rotation)> 150){
		sum_angle += (rotation);//- lastRotation);
		lastRotation = rotation;
	}else{
		lastRotation = 0;
	}
  //Serial.println(sum_angle);
  
  currentAngle = ((int)((((double)sum_angle)/950.741252))%36000)/100.0;//120274.1252
  if(currentAngle < 0) currentAngle += 360;
  //Serial.println(currentAngle);
 }

 int imu_postproccess_acc(){
	current_time = millis();
	long dt = current_time - last_time;
	accXd = ((accX - 500.0) * 9.81)/16500.0;
	if(abs(accXd) < 2){
		accXd = 0;
	}
	currentVXd = (accXd - lastAccXd)*dt;
	x_pos = x_pos + (currentVXd - lastVXd)*dt;
	lastVXd = currentVXd;
	lastAccXd = accXd;
	
	
	
	accYd = ((accY - 500.0) * 9.81)/16500.0;
	if(abs(accYd) < 2){
		accYd = 0;
	}
	currentVYd = (accYd - lastAccYd)*dt;
	y_pos = y_pos + (currentVYd - lastVYd)*dt;
	lastVYd = currentVYd;
	lastAccYd = accYd;
	
	last_time = current_time;
 }
 
 int imu_open_save_file(int save_count){
	char filename[100] = "";
	strcat(filename, data_filename);
	char str[15];
	sprintf(str, "%d", save_count);
	strcat(filename, str);
	strcat(filename, data_filename_end);
	Serial.println(filename);
	file = fopen(filename,"w+");
	return 0;
 }
 
 int imu_save_data() {
  
  fprintf(file, "%u,%d,%d,%f\n",millis(),accX,accY,currentAngle);
  fflush(file);
  return 0;
}

 int imu_close_save_file(){
	Serial.println("Closing");
	fclose(file);
	return 0;
 } 

 void init_filter(){
 #ifdef RESTRICT_PITCH // Eq. 25 and 26
	
	double roll  = atan2(accY, accZ) * RAD_TO_DEG;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	#else // Eq. 28 and 29
	double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
	#endif

    kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	gyroXangle = roll;
	gyroYangle = pitch;
	compAngleX = roll;
	compAngleY = pitch;

	timer = micros();
 }

 void loop_filter(){
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
 
 #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

	/*
 #if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
*/
 
 
 
 }
 
 void init_imu(){
// join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    //Serial.begin(9600);
    
    // initialize device
    imu.initialize();
    
    //Because it needed to be done
    imu.setSleepEnabled(true);
    imu.setSleepEnabled(false);
    
    /* Get I2C master clock speed.
     * I2C_MST_CLK is a 4 bit unsigned value which configures a divider on the
     * MPU-60X0 internal 8MHz clock. It sets the I2C master clock speed according to
     * the following table:
     *
     * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
     * ------------+------------------------+-------------------
     * 0           | 348kHz                 | 23
     * 1           | 333kHz                 | 24
     * 2           | 320kHz                 | 25
     * 3           | 308kHz                 | 26
     * 4           | 296kHz                 | 27
     * 5           | 286kHz                 | 28
     * 6           | 276kHz                 | 29
     * 7           | 267kHz                 | 30
     * 8           | 258kHz                 | 31
     * 9           | 500kHz                 | 16
     * 10          | 471kHz                 | 17
     * 11          | 444kHz                 | 18
     * 12          | 421kHz                 | 19
     * 13          | 400kHz                 | 20
     * 14          | 381kHz                 | 21
     * 15          | 364kHz                 | 22
     *
     * @return Current I2C master clock speed
     * @see MPU6050_RA_I2C_MST_CTRL
     */
    imu.setMasterClockSpeed(MPU6050_CLOCK_DIV_400);
	
	// Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
	// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    imu.setRate(7);
	
	/* Disable FSYNC
	 * 
	 * EXT_SYNC_SET | FSYNC Bit Location
	 * -------------+-------------------
	 * 0            | Input disabled
	 * 1            | TEMP_OUT_L[0]
	 * 2            | GYRO_XOUT_L[0]
	 * 3            | GYRO_YOUT_L[0]
	 * 4            | GYRO_ZOUT_L[0]
	 * 5            | ACCEL_XOUT_L[0]
	 * 6            | ACCEL_YOUT_L[0]
	 * 7            | ACCEL_ZOUT_L[0]
	 */
	imu.setExternalFrameSync(0);
	
	/*Set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
	 *          |   ACCELEROMETER    |           GYROSCOPE
	 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
	 * ---------+-----------+--------+-----------+--------+-------------
	 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
	 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
	 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
	 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
	 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
	 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
	 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
	 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
	 */
	imu.setDLPFMode(0);
	
	/* Set Gyro Full Scale Range to ±250deg/s 
	 * 0 = +/- 250 degrees/sec
	 * 1 = +/- 500 degrees/sec
	 * 2 = +/- 1000 degrees/sec
	 * 3 = +/- 2000 degrees/sec
	 */
	imu.setFullScaleGyroRange(0);
	
	/* Set Accelerometer Full Scale Range to ±2g
	 * 0 = +/- 2g
	 * 1 = +/- 4g
	 * 2 = +/- 8g
	 * 3 = +/- 16g
	 */
	imu.setFullScaleAccelRange(0);
	
	delay(100);
	
	imu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
	init_filter();
	
	p0 = get_data();
	
	
 }
 
 void loop_imu(){
   // read raw accel/gyro measurements from device
  imu.getMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ);
  
  loop_filter();
  imu_to_degrees();
  //imu_postproccess_acc();
  /*
  Serial.print(getAccX());
  Serial.print("\t");
  Serial.print(getAccY());
  Serial.print("\t");
  Serial.print(currentVXd);
  Serial.print("\t");
  Serial.print(currentVYd);
  Serial.print("\t");
  Serial.print(x_pos);
  Serial.print("\t");
  Serial.print(y_pos);
  Serial.println();
  */
  /*Serial.println(currentAngle);
	switch(save_data_code){
		case 0:
			imu_open_save_file();
		case 1:
			imu_save_data(); break;
		case 2:
			imu_close_save_file(); break;
		default:
			break;
 
	}
  */
 
 }
 
double getAccX() {return accXd;}
double getAccY() {return accYd;}
double getAccZ() {return accZ;}
double getGyroX(){return gyroX;}
double getGyroY(){return gyroY;}
double getGyroZ(){return gyroZ;}


 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 