
// il nome è rimasto mpu6050 ma il codice è relativo a mpu9255
///The MPU-9255 is actually 2 separate I2C devices, 
/// the accelerometer and gyro are accessed on I2C address 0x68 (or 0x69 depending on the logic level of the AD0 pin), 
/// the the magnetometer is accessed on I2C address 0x0C.



/**
MPU-9255 module (three-axis gyroscope + three-axis accelerometer + three-axis magnetic)
Chip:MPU-9255
Power supply:3-5v (internal low dropout regulator)
Communication:standard IIC communication protocol
Chip built-in 16bit AD converter,16-bit data output
Gyroscopes range: ±250 500 1000 2000 ° / s
Acceleration range: ±2 ± 4 ± 8 ± 16g
Field range: ±4800uT
Pitch:2.54mm
Module size:15mm * 25mm
**/

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "include/mpu9255/GyroAccel.h"
#include "include/mpu9255/Mag.h"

using namespace std;


/// I2C Addresses
const int I2C_ADDR_ACCGYRO = 0x68; /// gyro
const int I2C_ADDR_MAG = 0x0C; /// magnetometer


const int PWR_MGMT_1 = 0x6B;  /// equivale al registro 107 

float read_word_2c(int i2cH, int addr) {
  int high = wiringPiI2CReadReg8(i2cH, addr);
  int low = wiringPiI2CReadReg8(i2cH, addr+1);
  int val = (high << 8) + low;
  return float((val >= 0x8000) ? -((65535 - val) + 1) : val);
}

int main(int argc, char **argv) {

  /// Connect to acc gyro device.
  int i2cHAccGyro = wiringPiI2CSetup(I2C_ADDR_ACCGYRO);
  if (i2cHAccGyro == -1) {
    printf("no acc-gyro found!\n");
    return -1;
  }
  /// Connect to magnetometr device.
  int i2cHAccGyro = wiringPiI2CSetup(I2C_ADDR_MAG);
  if (i2cHAccGyro == -1) {
    printf("no magnetometer found!\n");
    return -1;
  }  
  
  // Device starts in sleep mode so wake it up.
  wiringPiI2CWriteReg16(i2cHAccGyro, PWR_MGMT_1, 0);


/// codice x mpu-9255
	//Start accel and gyro
	int32_t I2C_Handle_GyroAccel = initGyroAccel();
	if (I2C_Handle_GyroAccel < 0)
	{
		#ii2cHAccGyroef DEBUG
		std::cout << "Failed to init Gryo Accel" << std::endl;
		#endif
		return EXIT_FAILURE;
	}
	
	//Start magnetometer
	long double ASA[3];
	int32_t I2C_Handle_Mag = initMag(ASA);
	if (I2C_Handle_Mag < 0)
	{
		#ii2cHAccGyroef DEBUG
		std::cout << "Failed to init Mag" << std::endl;
		#endif
		return EXIT_FAILURE;
	}
/// ------------------------------------











  // Start ROS node stuff.
  ros::init(argc, argv, "mpu6050");
  ros::NodeHandle node;
  ros::Publisher pub = node.advertise<sensor_msgs::Imu>("/imu", 10);
  ros::Rate rate(10);  // hz

  // Publish in loop.
  while(ros::ok()) {
    sensor_msgs::Imu msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "top_link";  //'0'= no frame

    // Read gyroscope values.
    // At default sensitivity of 250deg/s we need to scale by 131.
    msg.angular_velocity.x = read_word_2c(i2cHAccGyro, 0x43) / 131;
    msg.angular_velocity.y = read_word_2c(i2cHAccGyro, 0x45) / 131;
    msg.angular_velocity.z = read_word_2c(i2cHAccGyro, 0x47) / 131;

    // Read accelerometer values.
    // At default sensitivity of 2g we need to scale by 16384.
    // Note: at "level" x = y = 0 but z = 1 (i.e. gravity)
    // But! Imu msg docs say acceleration should be in m/2 so need to *9.807
    const float la_rescale = 16384.0 / 9.807;
    msg.linear_acceleration.x = read_word_2c(i2cHAccGyro, 0x3b) / la_rescale;
    msg.linear_acceleration.y = read_word_2c(i2cHAccGyro, 0x3d) / la_rescale;
    msg.linear_acceleration.z = read_word_2c(i2cHAccGyro, 0x3f) / la_rescale;

    // Pub & sleep.
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
/*
#include <pigpio.h>
#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <cstdint>
#include <cinttypes>
#include <unistd.h>
#include <math.h> 
#include <fstream>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>

#define DEBUG 1
#define ROSUSLEEP 500

#include "include/mpu9255/GyroAccel.h"
#include "include/mpu9255/Mag.h"


int32_t main(int argc, char *argv[])
{
	#ii2cHAccGyroef DEBUG
	std::cout << "Starting" << std::endl;
	#endif
	//Start pigpio
	if (gpioInitialise() < 0)
	{
		#ii2cHAccGyroef DEBUG
		std::cout << "Failed to start" << std::endl;
		#endif
		return EXIT_FAILURE;
	}

	//Start accel and gyro
	int32_t I2C_Handle_GyroAccel = initGyroAccel();
	if (I2C_Handle_GyroAccel < 0)
	{
		#ii2cHAccGyroef DEBUG
		std::cout << "Failed to init Gryo Accel" << std::endl;
		#endif
		return EXIT_FAILURE;
	}
	
	//Start magnetometer
	long double ASA[3];
	int32_t I2C_Handle_Mag = initMag(ASA);
	if (I2C_Handle_Mag < 0)
	{
		#ii2cHAccGyroef DEBUG
		std::cout << "Failed to init Mag" << std::endl;
		#endif
		return EXIT_FAILURE;
	}
	
	//Setup ROS
	ros::init(argc, argv, "mpu9255_node");
	ros::NodeHandle n;
	ros::Publisher accel_pub = n.advertise<geometry_msgs::PointStamped>("accel", 1000);
	ros::Publisher gyro_pub = n.advertise<geometry_msgs::PointStamped>("gyro", 1000);
	ros::Publisher mag_pub = n.advertise<geometry_msgs::PointStamped>("mag", 100);
	ros::Rate loop_rate(2100);


	int16_t tempDestination[3];
	
	geometry_msgs::PointStamped accel;
	geometry_msgs::PointStamped mag;
	geometry_msgs::PointStamped gyro;
	while (ros::ok())
	{
		//read accel
		readAccelData(I2C_Handle_GyroAccel,tempDestination);
		accel.header.stamp = ros::Time::now();
		accel.point.x = ((double)tempDestination[0])*((double)(ACCELRES));
		accel.point.y = ((double)tempDestination[1])*((double)(ACCELRES));
		accel.point.z = ((double)tempDestination[2])*((double)(ACCELRES));
		accel_pub.publish(accel);

		usleep(ROSUSLEEP);

		//read gyro
		readGyroData(I2C_Handle_GyroAccel,tempDestination);
		gyro.header.stamp = ros::Time::now();
		gyro.point.x = ((double)tempDestination[0])*((double)(GYRORES));
		gyro.point.y = ((double)tempDestination[1])*((double)(GYRORES));
		gyro.point.z = ((double)tempDestination[2])*((double)(GYRORES));
		gyro_pub.publish(gyro);
		usleep(ROSUSLEEP);

		if ((i2cReadByteData(I2C_Handle_Mag,AK8963_ST1) & 0x01) == 0x0)
		{
			continue;
		}
		
		if (readMagData(I2C_Handle_Mag,tempDestination) == false )
		{
			continue;
		}
		//read mag
		mag.header.stamp = ros::Time::now();
		mag.point.x = ((double)tempDestination[0])*((double)ASA[0])*((double)MAGRES);
		mag.point.y = ((double)tempDestination[1])*((double)ASA[1])*((double)MAGRES);
		mag.point.z = ((double)tempDestination[2])*((double)ASA[2])*((double)MAGRES);
		mag_pub.publish(mag);

		usleep(ROSUSLEEP);
	}
	return EXIT_SUCCESS;

}
*/
