#include "include/mpu9255/Mag.h"


int32_t initMag(long double* factorySettings)
{
	int32_t I2C_Handle = i2cOpen( I2CBUS  ,MAGI2CADDR, 0);

	//Check
	if(I2C_Handle < 0)
	{
		#ifdef MAGDEBUG
		printf( "Failed to start I2C \n");
		#endif
		return -1;
	}

	//Check WHOAMI
	if (whoamiMag(I2C_Handle) != MAGWHOAMIVALUE)
	{
		#ifdef MAGDEBUG
		printf( "WHO AM I failed %d \n",whoamiMag(I2C_Handle));
		#endif
		return -1;
	}

	//Reset all values
	bool isValid = resetMag(I2C_Handle);
	if(isValid == false)
	{
		#ifdef MAGDEBUG
		printf( "Failed to reset \n");
		#endif
		return -1;
	}

	isValid = getFactorySettingsMag(I2C_Handle,factorySettings );
	if(isValid == false)
	{
		#ifdef MAGDEBUG
		printf( "Failed to get factory settings \n");
		#endif
		return -1;
	}
	
	//Check
	if (factorySettings == NULL)
	{
		#ifdef MAGDEBUG
		printf( "Failed to read factory settings \n");
		#endif
		return -1;
	}

	#ifdef MAGDEBUG
	printf( "Mag Factory settings ASA X:%Lf Y:%Lf Z:%Lf \n",factorySettings[0],factorySettings[1],factorySettings[2]);
	#endif

	//Setup configurations
	isValid = configureMag(I2C_Handle);
	if(isValid == false)
	{
		#ifdef MAGDEBUG
		printf( "Failed to configure \n");
		#endif
		return -1;
	}

	
	return I2C_Handle;
}



uint8_t whoamiMag(int32_t I2C_Handle)
{
	if (I2C_Handle < 0)
	{
		return 0x0;
	}

	//Get WHOAMI register
	int32_t isValid = i2cReadByteData(I2C_Handle,AK8963_WHO_AM_I);
	if (isValid < 0)
	{
		return 0x0;
	}
	return (uint8_t)isValid;
}


bool resetMag(int32_t I2C_Handle)
{
	if (I2C_Handle < 0)
	{
		return false;
	}
	
	//Reset everything
	int32_t isValid = i2cWriteByteData(I2C_Handle,AK8963_CNTL2,0b00000001);
	if (isValid < 0)
	{
		return false;
	}
	
	usleep(MAGSLEEPUS);
	
	//Clear registers
	isValid = i2cWriteByteData(I2C_Handle,AK8963_CNTL1,0b00000000);
	if (isValid < 0)
	{
		return false;
	}
	
	usleep(MAGSLEEPUS);
	
	return true;
}


bool getFactorySettingsMag(int32_t I2C_Handle,long double * destination)
{
	if (I2C_Handle < 0)
	{
		return false;
	}
	
	if (destination == NULL)
	{
		return false;
	}

		
	//Power down
	int32_t isValid = i2cWriteByteData(I2C_Handle,AK8963_CNTL1,0b00000000);
	if (isValid < 0)
	{
		return false;
	}
	
	usleep(MAGSLEEPUS);

	//Fuse ROM mode
	isValid = i2cWriteByteData(I2C_Handle,AK8963_CNTL1,0b00001111);
	if (isValid < 0)
	{
		return false;
	}
	
	usleep(MAGSLEEPUS);
	
	//Read factory settings
	char rawData[3];
	i2cReadI2CBlockData(I2C_Handle, AK8963_ASAX,rawData,3);
	destination[0] = (((long double)((uint8_t)rawData[0]) - 128.0 )/256.0) + 1.0 ;
	destination[1] = (((long double)((uint8_t)rawData[1]) - 128.0 )/256.0) + 1.0 ;
	destination[2] = (((long double)((uint8_t)rawData[2]) - 128.0 )/256.0) + 1.0 ;

	//Power down
	isValid = i2cWriteByteData(I2C_Handle,AK8963_CNTL1,0b00000000);
	if (isValid < 0)
	{
		return false;
	}
	
	usleep(MAGSLEEPUS);

	return true;
}

bool configureMag(int32_t I2C_Handle)
{
	if (I2C_Handle < 0)
	{
		return false;
	}

	//Continous measurement 8 Hz at 16 bit
	int32_t isValid = i2cWriteByteData(I2C_Handle, AK8963_CNTL1, 0b00010010); 
	if (isValid < 0)
	{
		return false;
	}
	
	usleep(MAGSLEEPUS);	

	return true;
}


bool readMagData(int32_t I2C_Handle,int16_t * destination)
{
	char rawData[7];
	i2cReadI2CBlockData(I2C_Handle, AK8963_HXL, rawData,7);
	uint8_t ST2 = (uint8_t)rawData[6];

	//Check for overflows
	if((ST2 & 0b00001000) == 0b00001000)
	{
		return false;
	}
	
	// Turn the MSB and LSB into a signed 16-bit value
	destination[0] = ((int16_t)((uint8_t)rawData[1]) << 8) | ((uint8_t)rawData[0]);
	// Data stored as little Endian 
	destination[1] = ((int16_t)((uint8_t)rawData[3]) << 8) | ((uint8_t)rawData[2]);
	destination[2] = ((int16_t)((uint8_t)rawData[5]) << 8) | ((uint8_t)rawData[4]);
	return true;
	
}


