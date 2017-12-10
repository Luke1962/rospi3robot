
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
#define MAGDEBUG 1

//I2C Address
#define MAGI2CADDR 0x0C

//Register Addresses
#define I2CBUS 1
#define MAGSLEEPUS 200000
#define MAGWHOAMIVALUE 0x48
#define AK8963_WHO_AM_I 0x00
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL2 0x0B
#define AK8963_ASAX 0x10
#define I2CDIS 0x0F
#define AK8963_HXL 0x03
#define AK8963_ST1 0x02


//Resolution in nT
#define MAGRES (149.93895)

/*
 * Function:  initMag 
 * --------------------
 * Starts I2C and writes registers to setup magnetometer
 *
 * factorySetting: Calibration constants from registers
 * 
 * returns: I2C handle
 */
int32_t initMag(long double* factorySetting);

/*
 * Function:  whoamiMag 
 * --------------------
 * Retrives the value of WHO_AM_I register
 *
 * I2C_Handle: Handle to access I2C
 * 
 * returns: I2C handle
 */
uint8_t whoamiMag(int32_t I2C_Handle);

/*
 * Function:  resetMag 
 * --------------------
 * Resets all registers
 *
 * I2C_Handle: Handle to access I2C
 * 
 * returns: I2C handle
 */
bool resetMag(int32_t I2C_Handle);

/*
 * Function:  getFactorySettingsMag
 * --------------------
 * Retrives factory calibration constants from registers
 *
 * I2C_Handle: Handle to access I2C
 *
 * destination: Destination to write factory constants 
 *
 * returns: Success of writing registers
 */
bool getFactorySettingsMag(int32_t I2C_Handle,long double * destination);

/*
 * Function:  configureMag
 * --------------------
 * Configure settings of magnetometer
 *
 * I2C_Handle: Handle to access I2C
 *
 * returns: Success of writing registers
 */
bool configureMag(int32_t I2C_Handle);

/*
 * Function:  readMagData
 * --------------------
 * Reads raw data from magnetometer registers 
 *
 * I2C_Handle: Handle to access I2C
 *
 * destination: Data of magnetometer registers
 *
 * returns: Success of writing registers
 */
bool readMagData(int32_t I2C_Handle,int16_t * destination);

