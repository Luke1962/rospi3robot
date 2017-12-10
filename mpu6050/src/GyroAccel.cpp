#include "include/mpu9255/GyroAccel.h"


int32_t initGyroAccel()
{
	int32_t I2C_Handle = i2cOpen( I2CBUS  ,GYROACCELI2CADDR, 0);

	//Check
	if(I2C_Handle < 0)
	{
		#ifdef GYROACCELDEBUG
		printf( "Failed to start I2C \n");
		#endif
		return -1;
	}

	//Check WHOAMI
	if (whoamiGyroAccel(I2C_Handle) != GYROACCELWHOAMIVALUE)
	{
		#ifdef GYROACCELDEBUG
		printf( "WHO AM I failed \n");
		#endif
		return -1;
	}

	//Reset all values
	bool isValid = resetGyroAccel(I2C_Handle);
	if(isValid == false)
	{
		#ifdef GYROACCELDEBUG
		printf( "Failed to reset \n");
		#endif
		return -1;
	}


	//Setup configurations
	isValid = configureGyroAccel(I2C_Handle);
	if(isValid == false)
	{
		#ifdef GYROACCELDEBUG
		printf( "Failed to configure \n");
		#endif
		return -1;
	}

	
	//Calibrate gyro
	int16_t offset[3];
	isValid = readGyroCal(I2C_Handle,offset);	

	if(isValid == false)
	{
		#ifdef GYROACCELDEBUG
		printf( "Failed to read gyro calibration \n");
		#endif
		return -1;
	}

	//Check
	if (offset == NULL)
	{
		#ifdef GYROACCELDEBUG
		printf( "Failed to read gyro calibration \n");
		#endif
		return -1;
	}

	#ifdef GYROACCELDEBUG
	printf( "Gyro offsets X:%d Y:%d Z:%d \n",offset[0],offset[1],offset[2]);
	#endif

	usleep(GYROACCELSLEEPUS);

	//Write gyro calibration
	isValid = writeGyroCal(I2C_Handle,offset);
	if(isValid == false)
	{
		#ifdef GYROACCELDEBUG
		printf( "Failed to write gyro calibration  \n");
		#endif
		return -1;
	}
	usleep(GYROACCELSLEEPUS);
	



	
	//Calibrate Accelerometer
	isValid = readAccelCal(I2C_Handle,offset);

	if(isValid == false)
	{
		#ifdef GYROACCELDEBUG
		printf( "Failed to read accel calibration \n");
		#endif
		return -1;
	}

	if (offset == NULL)
	{
		#ifdef GYROACCELDEBUG
		printf( "Failed to read accel calibration \n");
		#endif
		return -1;
	}

	//Remove gravity
	offset[2] -= (int16_t)ZNORMAL;
	
	#ifdef GYROACCELDEBUG
	printf( "Accel offset X:%d Y:%d Z:%d \n",offset[0],offset[1],offset[2]);
	#endif
	
	usleep(GYROACCELSLEEPUS);

	//Write accel calibration
	isValid = writeAccelCal(I2C_Handle,offset);
	if(isValid == false)
	{
		#ifdef GYROACCELDEBUG
		printf( "Failed to write gyro calibration  \n");
		#endif
		return -1;
	}

	usleep(GYROACCELSLEEPUS);

	
	return I2C_Handle;
}

uint8_t whoamiGyroAccel(int32_t I2C_Handle)
{
	if (I2C_Handle < 0)
	{
		return 0x0;
	}

	//Get WHOAMI register
	int32_t isValid = i2cReadByteData(I2C_Handle,WHO_AM_I);
	if (isValid < 0)
	{
		return 0x0;
	}
	return (uint8_t)isValid;
}


bool resetGyroAccel(int32_t I2C_Handle)
{
	if (I2C_Handle < 0)
	{
		return false;
	}
	
	//Reset everything
	int32_t isValid = i2cWriteByteData(I2C_Handle,PWR_MGMT_1,0b10000000);
	if (isValid < 0)
	{
		return false;
	}
	
	usleep(GYROACCELSLEEPUS);
	
	//Reset CLOCK
	isValid = i2cWriteByteData(I2C_Handle,PWR_MGMT_1,0b00000001);
	if (isValid < 0)
	{
		return false;
	}
	//turn on all sensors
	isValid = i2cWriteByteData(I2C_Handle,PWR_MGMT_2,0b00000000);
	if (isValid < 0)
	{
		return false;
	}
	
	usleep(GYROACCELSLEEPUS);

	i2cWriteByteData(I2C_Handle, INT_ENABLE, 0x00);   // Disable all interrupts
	i2cWriteByteData(I2C_Handle, INT_PIN_CFG, 0x00);
	i2cWriteByteData(I2C_Handle, FIFO_EN, 0x00);      // Disable FIFO
	i2cWriteByteData(I2C_Handle, I2C_MST_CTRL, 0x00); // Disable I2C master
	i2cWriteByteData(I2C_Handle, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes

	usleep(GYROACCELSLEEPUS);
	return true;
}

bool configureGyroAccel(int32_t I2C_Handle)
{
	if (I2C_Handle < 0)
	{
		return false;
	}
	
	//Set Gyro to 32kHz 
	//FCHOICE_B = 11
	/*
	int32_t isValid = i2cWriteByteData(I2C_Handle,GYRO_CONFIG,0b00000011);
	if (isValid < 0)
	{
		return false;
	}
	*/

	/*
	//set Gyro to 8kHz
	int32_t isValid = i2cWriteByteData(I2C_Handle,GYRO_CONFIG,0b00000000);
	if (isValid < 0)
	{
		return false;
	}
	

	//Get accelerometer to 4kHz
	isValid = i2cWriteByteData(I2C_Handle,ACCEL_CONFIG_2,0b00001100);
	if (isValid < 0)
	{
		return false;
	}
	*/

	//Gyro 1 kHz
	int32_t isValid = i2cWriteByteData(I2C_Handle,CONFIG,0b00000001);
	if (isValid < 0)
	{
		return false;
	}

	//Acceleromete to 1 kHz
	isValid = i2cWriteByteData(I2C_Handle,ACCEL_CONFIG_2,0b00000000);
	if (isValid < 0)
	{
		return false;
	}
	
	usleep(GYROACCELSLEEPUS);

	//Enable I2C bypass for magnometer
	isValid = i2cWriteByteData(I2C_Handle,INT_PIN_CFG,0b00000010);
	if (isValid < 0)
	{
		return false;
	}
	usleep(GYROACCELSLEEPUS);

	return true;
}

bool ISRGyroAccel(int32_t I2C_Handle, gpioISRFuncEx_t f,void *userdata)
{

	if (I2C_Handle < 0)
	{
		return false;
	}
	
	if (f == NULL)
	{
		return false;
	}

	if (userdata == NULL)
	{
		return false;
	}

	//Set ISR
	if (gpioSetISRFuncEx(INTPIN,INTTYPE,ISRPINTIMEOUT,f,userdata) != SETGPIOISROK)
	{
		return false;
	}

	//Set latch and auto clear
	int isValid = i2cWriteByteData(I2C_Handle,INT_PIN_CFG,0b00110010);
	if (isValid < 0)
	{
		return false;
	}
	usleep(GYROACCELSLEEPUS);
	
	//Enable raw interrupts
	isValid = i2cWriteByteData(I2C_Handle,INT_ENABLE,0b00000001);
	if (isValid < 0)
	{
		return false;
	}
	usleep(GYROACCELSLEEPUS);
		
	return true;
}


bool readAccelCal(int32_t I2C_Handle,int16_t * offset)
{
	if (I2C_Handle < 0)
	{
		return false;
	}

	if (offset == NULL)
	{
		return false;
	}

	uint32_t i;
	int16_t tempDestination[3];
	int64_t offset_temp[3] = {0,0,0};
	
	//Add all samples
	for(i = 0;i<CALACCELSAMPLE;++i)
	{
		readAccelData(I2C_Handle,tempDestination);
		offset_temp[0] += tempDestination[0];
		offset_temp[1] += tempDestination[1];
		offset_temp[2] += tempDestination[2];
	}

	//Average
	offset[0] = (int16_t)(offset_temp[0]/((int64_t)CALACCELSAMPLE));
	offset[1] = (int16_t)(offset_temp[1]/((int64_t)CALACCELSAMPLE));
	offset[2] = (int16_t)(offset_temp[2]/((int64_t)CALACCELSAMPLE));

	return true;	
}



bool writeAccelCal(int32_t I2C_Handle,int16_t * offset)
{
	if (I2C_Handle < 0)
	{
		return false;
	}

	if (offset == NULL)
	{
		return false;
	}
	
	//Get offsets
	uint8_t calRegister[6];
	int32_t accel_bias_reg[3] = {0, 0, 0};	
	calRegister[0] = (uint8_t)i2cReadByteData(I2C_Handle, XA_OFFSET_H);
	calRegister[1] = (uint8_t)i2cReadByteData(I2C_Handle, XA_OFFSET_L);
	accel_bias_reg[0] = (int32_t) (((int16_t)calRegister[0] << 8) | calRegister[1]);
	calRegister[0] = (uint8_t)i2cReadByteData(I2C_Handle, YA_OFFSET_H);
	calRegister[1] = (uint8_t)i2cReadByteData(I2C_Handle, YA_OFFSET_L);
	accel_bias_reg[1] = (int32_t) (((int16_t)calRegister[0] << 8) | calRegister[1]);
	calRegister[0] = (uint8_t)i2cReadByteData(I2C_Handle, ZA_OFFSET_H);
	calRegister[1] = (uint8_t)i2cReadByteData(I2C_Handle, ZA_OFFSET_L);
	accel_bias_reg[2] = (int32_t) (((int16_t)calRegister[0] << 8) | calRegister[1]);
	
	/*	
	uint32_t mask = 0x01; 
	uint8_t mask_bit[3] = {0, 0, 0}; 
	uint8_t ii;
	for(ii = 0; ii < 3; ii++) {
		if((accel_bias_reg[ii] & mask)) 
		{
			mask_bit[ii] = 0x01;
		} 
	}
	*/
	
	//Stop temperature compensation.	

	accel_bias_reg[0] -= (offset[0]/8); 
	accel_bias_reg[1] -= (offset[1]/8); 
	accel_bias_reg[2] -= (offset[2]/8);

	calRegister[0] = (accel_bias_reg[0]  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format	
	calRegister[1] = (accel_bias_reg[0])       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	//calRegister[1] = calRegister[1] | mask_bit[0];
	calRegister[1] = calRegister[1] & 0b11111110;

	calRegister[2] = (accel_bias_reg[1]  >> 8) & 0xFF;
	calRegister[3] = (accel_bias_reg[1])       & 0xFF;
	//calRegister[3] = calRegister[3] | mask_bit[1]; 
	calRegister[3] = calRegister[3]  & 0b11111110;

	calRegister[4] = (accel_bias_reg[2]  >> 8) & 0xFF;
	calRegister[5] = (accel_bias_reg[2]) & 0xFF;
	//calRegister[5] = calRegister[5] | mask_bit[2]; 
	
	calRegister[5] = calRegister[5] & 0b11111110;

	//Write to register
	i2cWriteByteData(I2C_Handle, XA_OFFSET_H, calRegister[0]);
	i2cWriteByteData(I2C_Handle, XA_OFFSET_L, calRegister[1]);
	i2cWriteByteData(I2C_Handle, YA_OFFSET_H, calRegister[2]);
	i2cWriteByteData(I2C_Handle, YA_OFFSET_L, calRegister[3]);
	i2cWriteByteData(I2C_Handle, ZA_OFFSET_H, calRegister[4]);
	i2cWriteByteData(I2C_Handle, ZA_OFFSET_L, calRegister[5]);


	//Check one register
	uint8_t XA_OFFSET_H_REG = (uint8_t)i2cReadByteData(I2C_Handle, XA_OFFSET_H);
	if (XA_OFFSET_H_REG != calRegister[0])
	{
		return false;
	}
	return true;	
	
}

bool readGyroCal(int32_t I2C_Handle,int16_t * offset)
{
	if (I2C_Handle < 0)
	{
		return false;
	}

	if (offset == NULL)
	{
		return false;
	}

	uint32_t i;
	int16_t tempDestination[3];
	int64_t offset_temp[3] = {0,0,0};
	
	//Add all samples
	for(i = 0;i<CALGYROSAMPLE;++i)
	{
		readGyroData(I2C_Handle,tempDestination);
		offset_temp[0] += tempDestination[0];
		offset_temp[1] += tempDestination[1];
		offset_temp[2] += tempDestination[2];
	}

	//Average
	offset[0] = (int16_t)(offset_temp[0]/((int64_t)CALGYROSAMPLE));
	offset[1] = (int16_t)(offset_temp[1]/((int64_t)CALGYROSAMPLE));
	offset[2] = (int16_t)(offset_temp[2]/((int64_t)CALGYROSAMPLE));
	
	return true;
}

bool writeGyroCal(int32_t I2C_Handle,int16_t * offset)
{
	if (I2C_Handle < 0)
	{
		return false;
	}

	if (offset == NULL)
	{
		return false;
	}

	uint8_t calRegister[6];
	calRegister[0] = (-offset[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format	
	calRegister[1] = (-offset[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	calRegister[2] = (-offset[1]/4  >> 8) & 0xFF;
	calRegister[3] = (-offset[1]/4)       & 0xFF;
	calRegister[4] = (-offset[2]/4  >> 8) & 0xFF;
	calRegister[5] = (-offset[2]/4) & 0xFF;

	//Write to register	
	i2cWriteByteData(I2C_Handle, XG_OFFSET_H, calRegister[0]);
	i2cWriteByteData(I2C_Handle, XG_OFFSET_L, calRegister[1]);
	i2cWriteByteData(I2C_Handle, YG_OFFSET_H, calRegister[2]);
	i2cWriteByteData(I2C_Handle, YG_OFFSET_L, calRegister[3]);
	i2cWriteByteData(I2C_Handle, ZG_OFFSET_H, calRegister[4]);
	i2cWriteByteData(I2C_Handle, ZG_OFFSET_L, calRegister[5]);


	//Check one register
	uint8_t XG_OFFSET_H_REG = (uint8_t)i2cReadByteData(I2C_Handle, XG_OFFSET_H);
	if (XG_OFFSET_H_REG != calRegister[0])
	{
		return false;
	}
	
	return true;	
}


void readGyroData(int32_t I2C_Handle,int16_t * destination)
{
	char rawData[6];
	i2cReadI2CBlockData(I2C_Handle, GYRO_XOUT_H,rawData,6);
	destination[0] = ((int16_t)((uint8_t)rawData[0]) << 8) | ((uint8_t)rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)((uint8_t)rawData[2]) << 8) | ((uint8_t)rawData[3]) ;  
	destination[2] = ((int16_t)((uint8_t)rawData[4]) << 8) | ((uint8_t)rawData[5]) ;
}

void readAccelData(int32_t I2C_Handle,int16_t * destination)
{
	char rawData[6];
	i2cReadI2CBlockData(I2C_Handle, ACCEL_XOUT_H,rawData,6);
	destination[0] = ((int16_t)((uint8_t)rawData[0]) << 8) | ((uint8_t)rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = ((int16_t)((uint8_t)rawData[2]) << 8) | ((uint8_t)rawData[3]) ;  
	destination[2] = ((int16_t)((uint8_t)rawData[4]) << 8) | ((uint8_t)rawData[5]) ;
}


