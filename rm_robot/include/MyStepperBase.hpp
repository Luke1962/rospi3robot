#if !defined(__MYSTEPPERBASE_H__)
#define __MYSTEPPERBASE_H__

#include "parameters.h"
#include "hw_config.hpp"
#include "dbg.h"
#include "msgCore.h"
////#include <digitalWrite\digitalWrite.h>

#include <wiringPi.h>
#include <softTone.h>

#include <math.h> /// per fabs()
#include <stdlib.h>
enum motorDir_e { 
	DIR_S =0 ,	//S=STOP
	DIR_F,	// Forward 
	DIR_B,	//Backward
	DIR_R,	// Right
	DIR_L	// Left 
};


typedef unsigned long motCk_t; // was typedef uint32_t motCk; 
struct StepperDriver_t {
	motCk_t	clock; // clock motori in microsecondi, limitato tra ROBOT_MOTOR_CLOCK_microsecondMIN/MAX
	bool	enL;		//enable
	bool	enR;
	bool	cwL;		// direzione 1 = CW
	bool	cwR;
	bool    isMoving;
	//int acceleration;	// valore dell'acceleraz. in incrementi di CK in microsecondi
	//int accelPhase;		// 1= in acc. , 0=velocità costante -1, in decel.
	//int accelSteps;	// ampiezza della fase di accel/decel. in step
	//volatile	uint16_t stepsDone;		//step percorsi, aggiornato dalla lettura encoders
	//uint16_t targetSteps;	//step da percorrere 
	float targetVelocityLinear;		// velocità finale in m/s
	float targetVelocityAngular;		// velocità di rotazione in rad/s
	double targetVelocityLastTime; // millis() dell'ultimo comando
	//long targetEncoderThicks; //numero di thick dell'encoder
	//motCk_t targetCK;	//velocità finale in CK
	//int targetCm;
	//long timeStamp; // millis() a cui risale il comando
//	commandStatus_e commandStatus; //comando in corso di esecuzione o terminato
	motorDir_e commandDir;
	//position_t poseTarget;	// posizione obiettivo nelle coordinate World

};


class MyStepperBase_c
{
public:
	MyStepperBase_c();
	void init();
	//MyStepperBase_c(int pinCK, int pinEn, int pinCw, int pinStop, int microstepDivider = 2);
	//void updatePoseFromEncoders();
	void goCmdVel(float targetVelocityLinear, float targetVelocityAngular);
	void stop();
	motorDir_e getCmdDir();
	char getDirStr();
	StepperDriver_t cmd;  //contiene commandDir
	bool blSpeedLimiterOn ;
private:
//	motorDir_e _commandDir;
	void _goHz(motorDir_e dir, float stepsPerSecond);
	int _motorClockLimiter(int ckInterval); // limita il clock tra i due limiti ROBOT_MOTOR_CLOCK_microsecondMAX  e ROBOT_MOTOR_CLOCK_microsecondMIN
	void _motorSetPWMCK(float herz);
	void _setMotorDir(motorDir_e dir);



};
/*
Libreria per gli stepper della base mobile del robot
*/


MyStepperBase_c::MyStepperBase_c()
{
	init();
	stop();
}
void MyStepperBase_c::init()
{
	cmd.targetVelocityLinear = 0.0;
	cmd.targetVelocityAngular = 0.0;
	cmd.targetVelocityAngular = 0;
	cmd.commandDir = DIR_S ; //stop;
	//~ softToneCreate(Pin_MotCK);/// pin numbering will be that of the wiringPiSetup() function you used.
	blSpeedLimiterOn = true;
}

int MyStepperBase_c::_motorClockLimiter(int stepPersecond) // limita il clock tra i due limiti ROBOT_MOTOR_CLOCK_microsecondMAX  e ROBOT_MOTOR_CLOCK_microsecondMIN
{
	// controlla che il clock sia entro i limiti
	if (blSpeedLimiterOn)
	{
		if (stepPersecond > ROBOT_MOTOR_SPEED_MAX) {
			return ROBOT_MOTOR_SPEED_MAX;
		}
		else if (stepPersecond < ROBOT_MOTOR_SPEED_MIN) {
			return ROBOT_MOTOR_SPEED_MIN;
		}
		else {
			return stepPersecond;	//ok > registro il nuovo valore di clock
		};	
	}else
	{
		return stepPersecond;
	}

}
/**
  motCk_t MyStepperBase_c::_motorClockLimiter(motCk_t ckInterval) // limita il clock tra i due limiti ROBOT_MOTOR_CLOCK_microsecondMAX  e ROBOT_MOTOR_CLOCK_microsecondMIN
{
	// controlla che il clock sia entro i limiti
	if (blSpeedLimiterOn)
	{
		if (ckInterval > ROBOT_MOTOR_CLOCK_microsecondMAX) {
			return ROBOT_MOTOR_CLOCK_microsecondMAX;
		}
		else if (ckInterval < ROBOT_MOTOR_CLOCK_microsecondMIN) {
			return ROBOT_MOTOR_CLOCK_microsecondMIN;
		}
		else {
			return ckInterval;	//ok > registro il nuovo valore di clock
		};	
	}else
	{
		return ckInterval;
	}

}
 */
 
 
/// assegna il clock in base alla frequenza controllando i limiti
void MyStepperBase_c::_motorSetPWMCK(float herz){
	cmd.clock = _motorClockLimiter( round( herz));

	//ckFrequencySafe = 1000000.0 / cmd.clock;
	dbg2( "requested Hz: ", herz ) ;
	dbg2("   assigned Hz: ", cmd.clock );


	///WiringPi: 
	//~ softToneCreate(Pin_MotCK);
	//~ softToneWrite (Pin_MotCK, cmd.clock ) ; /// wiringpi
	
	/// interfaccia col thread
	g_motorCk =herz;
	g_motorCkHalfPeriodUs = cmd.clock /2;
};

motorDir_e MyStepperBase_c::getCmdDir() {
	return cmd.commandDir;
}
// imposta solamente status.cmd.cwL e status.cmd.cwR
void MyStepperBase_c::_setMotorDir(motorDir_e dir) {
	cmd.commandDir = dir;
	switch (dir)
	{
	case DIR_F:
		cmd.cwL = true;
		cmd.cwR = false;
		break;
	case DIR_B:
		cmd.cwL = false;
		cmd.cwR = true;
		break;
	case DIR_R:
		cmd.cwL = true;
		cmd.cwR = true;
		break;
	case DIR_L:
		cmd.cwL = false;
		cmd.cwR = false;
		break;
	case DIR_S:
	default:
		break;
	}
	if (cmd.cwR)
	{
		digitalWrite(Pin_MotCWR, ROBOT_MOTORCW_ACTIVE);	//LOW = CW	(ok)
	}
	else
	{
		digitalWrite(Pin_MotCWR, !ROBOT_MOTORCW_ACTIVE);	//LOW = CW	(ok)
	}
	if (cmd.cwL)
	{
		digitalWrite(Pin_MotCWL, ROBOT_MOTORCW_ACTIVE);	//LOW = CW	(ok)
	}
	else
	{
		digitalWrite(Pin_MotCWL, !ROBOT_MOTORCW_ACTIVE);	//LOW = CW	(ok)
	}


}

// ritorna la direzione corrente in formato stringa
char MyStepperBase_c::getDirStr() {

	switch (cmd.commandDir)
	{
	case DIR_F:
		return 'F';
		break;
	case DIR_B:
		return 'B';
		break;
	case DIR_R:
		return 'R';
		break;
	case DIR_L:
		return 'L';
		break;
	case DIR_S:
		return 'S';
		break;
	default:
		return '?';
		break;
	}

}

// ckIntervalMicroseconds: clock motori (6000 - 2500) >> (166,7Hz - 400 Hz)
void MyStepperBase_c::_goHz(motorDir_e dir, float stepsPerSecond) {


	_setMotorDir(dir);

	_motorSetPWMCK(stepsPerSecond);
	cmd.isMoving = true;	// segnala che si sta muovendo
	MOTORS_ENABLE
}


////////////////////////////////////////////////////////////////////
// interfaccia ROS
// memorizza le velocità rischieste
// in cmd.targetVelocityLinear e ..Angular
// in base ai valori di velocità, imposta cmdDir
////////////////////////////////////////////////////////////////////
// Vmin = Hzmin/ROBOT_M2STEPS = 166.7/2000 = 0.083 m/s
// Vmax =HzMax/ROBOT_M2STEPS =  400 /2000 = 0.2 m/s
// Vang_min =Hzmin/CMDVEL_ANGULAR_TO_MOTORHZ  =  166.7  /341,992 =0.487 rad/sec
// Vang_Max =HzMax/CMDVEL_ANGULAR_TO_MOTORHZ  =   400 /341,992 =1.17 rad/sec

void MyStepperBase_c::goCmdVel(float targetVelocityLinear, float targetVelocityAngular) 
{
	dbg2("targetVelocityAngular: ",targetVelocityAngular);
	dbg2("targetVelocityLinear: ",targetVelocityLinear);
	float ck;

	cmd.targetVelocityLinear = targetVelocityLinear;
	cmd.targetVelocityAngular = targetVelocityAngular;

	if (abs(cmd.targetVelocityLinear) > 0.001 )
	{

		/// ck INTERO in microsecondi
		ck = (int)(fabs(cmd.targetVelocityLinear) * ROBOT_M2STEPS);
		dbg2("  ck lin: ", ck);
		if (cmd.targetVelocityLinear > 0)
		{
			/// FORWARD
			_goHz(DIR_F, ck);
			//MSG("F");
		}
		else
		{
			//BACK
			_goHz(DIR_B, ck);
			//MSG("B");

		}

	}
	else if (fabs( cmd.targetVelocityAngular) > 0.001)  // comando di ANGULAR VELOCITY
	{
		ck = (int)(fabs(cmd.targetVelocityAngular)*ROBOT_RAD2STEPS);
		dbg2(" ck ang: ", ck);

	


		if (cmd.targetVelocityAngular > 0)
		{
			//ccw
			_goHz(DIR_L, ck);
			//MSG("L");

		}
		else
		{
			//cw
			_goHz(DIR_R, ck);
			//MSG("R");

		}

	}
	else
	{
		stop();

	}


}


//////////////////////////////////////////////////////
// Ferma i motori disabilitando gli avvolgimenti
void MyStepperBase_c::stop() {
	cmd.commandDir = DIR_S;
	cmd.enL = 0;
	cmd.enR = 0;

	MOTORS_DISABLE;
	////noTone(Pin_MotCK);
	//softToneWrite (Pin_MotCK, 0) ; /// wiringpi

	cmd.isMoving = false;	// segnala che si sta muovendo

};


//extern MyStepperBase_c robotMotors;
#endif
