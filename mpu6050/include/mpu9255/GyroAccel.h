#include <pigpio.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>
#include <math.h> 
#include <stdint.h>
#include <stdbool.h>

//Debug print out
#define GYROACCELDEBUG 1

//I2C Address
#define GYROACCELI2CADDR 0x68

//Sleep time
#define GYROACCELSLEEPUS 200000

//Interrupt settings
#define INTPIN 4
#define INTTYPE RISING_EDGE
#define SETGPIOISROK 0
#define ISRPINTIMEOUT 0


//Resolution
// UNIT m/s^2
#define ACCELRES (0.000598754883)
// UNIT rad/s
#define GYRORES  (0.000133158055)


//Calibration samples
#define CALGYROSAMPLE 1000
#define CALACCELSAMPLE 1000
#define ZNORMAL 16384


//Register Addresses
#define I2CBUS 1
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG_2 0x1D
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A
#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define WHO_AM_I 0x75
#define GYROACCELWHOAMIVALUE 0x73


#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define XG_OFFSET_H 0x13
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18

#define XA_OFFSET_H 0x77
#define XA_OFFSET_L 0x78
#define YA_OFFSET_H 0x7A
#define YA_OFFSET_L 0x7B
#define ZA_OFFSET_H 0x7D
#define ZA_OFFSET_L 0x7E

/*
 * Function:  initGyroAccel 
 * --------------------
 * Starts I2C and writes registers to setup gyroscope and accelerometer
 *
 * returns: I2C handle
 */
int32_t initGyroAccel();


/*
 * Function:  whoamiGyroAccel
 * --------------------
 * Retrives the value of WHO_AM_I register
 *
 * I2C_Handle: Handle to access I2C
 *
 * returns: Value of WHO_AM_I register 
 */
uint8_t whoamiGyroAccel(int32_t I2C_Handle);


/*
 * Function:  resetGyroAccel
 * --------------------
 * Resets all registers
 *
 * I2C_Handle: Handle to access I2C
 *
 * returns: Success of writing registers
 */
bool resetGyroAccel(int32_t I2C_Handle);

/*
 * Function:  configureGyroAccel
 * --------------------
 * Configure settings of gyroscope and accelerometer 
 *
 * I2C_Handle: Handle to access I2C
 *
 * returns: Success of writing registers
 */
bool configureGyroAccel(int32_t I2C_Handle);


/*
 * Function:  ISRGyroAccel
 * --------------------
 * Configures interrupt for gyroscope and accelerometer  
 *
 * I2C_Handle: Handle to access I2C
 *
 * f: Function to handle interrupt 
 *
 * userdata: Data to pass into the function
 *
 * returns: Success of writing registers
 */
bool ISRGyroAccel(int32_t I2C_Handle, gpioISRFuncEx_t f,void *userdata);


/*
 * Function:  readAccelCal
 * --------------------
 * Retrives the offset of accelerometer 
 *
 * I2C_Handle: Handle to access I2C
 *
 * offset: Data of offset
 *
 * returns: Success of reading registers
 */
bool readAccelCal(int32_t I2C_Handle,int16_t * offset);


/*
 * Function:  writeAccelCal
 * --------------------
 * Writes offset data into accelerometer registers
 *
 * I2C_Handle: Handle to access I2C
 *
 * offset: Data of offset
 *
 * returns: Success of writing registers
 */
bool writeAccelCal(int32_t I2C_Handle,int16_t * offset);

/*
 * Function:  readGyroCal
 * --------------------
 * Retrives the offset of gyroscope
 *
 * I2C_Handle: Handle to access I2C
 *
 * offset: Data of offset
 *
 * returns: Success of reading registers
 */
bool readGyroCal(int32_t I2C_Handle,int16_t * offset);

/*
 * Function:  writeGyroCal
 * --------------------
 * Writes offset data into gyroscope registers
 *
 * I2C_Handle: Handle to access I2C
 *
 * offset: Data of offset
 *
 * returns: Success of writing registers
 */
bool writeGyroCal(int32_t I2C_Handle,int16_t * offset);

/*
 * Function:  readGyroData
 * --------------------
 * Reads raw data from gyroscope registers 
 *
 * I2C_Handle: Handle to access I2C
 *
 * destination: Data of gyroscope registers
*/
void readGyroData(int32_t I2C_Handle,int16_t * destination);

/*
 * Function:  readAccelData
 * --------------------
 * Reads raw data from accelerometer registers 
 *
 * I2C_Handle: Handle to access I2C
 *
 * destination: Data of accelerometer registers
 */
void readAccelData(int32_t I2C_Handle,int16_t * destination);

