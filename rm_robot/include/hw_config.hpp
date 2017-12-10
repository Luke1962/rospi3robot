/*
 * hw_config.h
 * Include di base del robot
 * Created: 25/01/2015 16:53:17
 *  Author: Luca
 */ 
 #include <wiringPi.h>
#ifndef HW_CONFIG_H_
#define HW_CONFIG_H_

//////////////////////////////////////////////////////////////////////////
//																		//
//						 O P Z I O N I   R O B O T						//
//																		//
//////////////////////////////////////////////////////////////////////////


#define OPT_COMPASS 1
#define OPT_MPU6050 1
#define OPT_SONAR 0
#define OPT_SERVOSONAR 0	//INCLUDI FUNZIONALITA' SERVO-SONAR
#define OPT_ENCODERS  1	//INCLUDI ENCODERS
#define OPT_GPS 0
#define OPT_LDS 0	//Laser Distance sensor
#define OPT_STEPPERLDS 0
//#define OPT_BT 0	//bluetooth
//#define OPT_SPEECH 0


// parametri ficici del robot---------------
#define	STEPSPERCM 23.342725f 
#define	MMPERSTEPS  0.42840f 


// parametri stepper ruote---------------
#define ROBOT_MOTORENABLE_ACTIVE 0		// 1 = enable a logica positiva 0 = enable a logica negata (DRV8825)
#define ROBOT_MOTORCW_ACTIVE 0		// 1 = CW a logica positiva 0 = CW a logica negata (DRV8825)



//----------------------------------------------
////////////////////////////////////////////////
// NOTAZIONE wiringpi 
/////////////////////////////////////////////////

/// ----------------------------------------------
/// pin da configurare esplicitamente in OUTPUT				----
/// ----------------------------------------------


#define Pin_MotENR 6	/// RIGHT MOTOR ENABLE
#define Pin_MotCWR 10	///  left CW  - 24 RIGHT MOTOR
#define Pin_MotCK  11 //3//	solo per test , 11		///=FREQUENCYTIMER2_PIN=10	// CLOCK  << vedere in funzione del timer usato Timer 3 =>pin A=5, B=2, C=3

#define Pin_MotENL 27	/// LEFT MOTOR ENABLE
#define Pin_MotCWL 28	/// RIGHT MOTOR 1=CW  0=CCW
#define Pin_MotCKL 29 

#define  Pin_LED_TOP_R 0	
#define  Pin_LED_TOP_G 2
#define  Pin_LED_TOP_B 3

#define Pin_LaserOn 1	/// 23 Accensione Laser 1=CW  0=CCW


////#define Pin_SonarTrig 	// pin TRIGGER vs Sonar
////#define Pin_Rele1	
////#define Pin_Rele2	

/// ----------------------------------------------
/// pin da configurare in INPUT -----------------
/// ----------------------------------------------

#define Pin_EncRa	4	//		// encoder Right Motor segnale a	
#define Pin_EncLa	5			// encoder Left Motor segnale a	

#define Pin_irproxy_FW 21		// sensore di prossimità anteriore
#define Pin_irproxy_FWHL 22		// sensore di prossimità anteriore per alta luce
#define Pin_irproxy_BK 23		// sensore di prossimità POSTERIORE
#define Pin_irproxy_FR 24	// IR Proxy anteriore Destro
#define Pin_irproxy_FL 25	// IR Proxy anteriore sinistro

////#define Pin_PIR1	27			// Sensore presenza umana
////#define Pin_BtState 40

////#define Pin_EncRb	31			// rimosso
////#define Pin_SonarEcho 32		// pin ECHO da Sonar
////#define Pin_EncLb	33			// encoder Left Motor segnale b

/// ----------------------------------------------
/// pin da configurare in INPUT PULLUP	--------
/// ----------------------------------------------
#define Pin_SDA 8
#define Pin_SCL 9

#define Pin_BumbRight 12
#define Pin_BumbCenter 13
#define Pin_BumbLeft 14

#define Pin_SwitchTop  26	// deviatore vicino arduino

////#define PIN_STEPPERLDS_HOME	A8	// A3
////#define PIN_STEPPERLDS_END	A9	// A3



///INGRESSI ANALOGICI----------------------------------------------
///  *** NON DISPONIBILI SU PI3 ***
////#define Pin_AnaBase 54 // A0 =54 per recuperara l'indice dell'array uso [Pin_AnaBase -Pin_AnaLight]
////#define Pin_AnaVbat A0		// Potenziometro che regola la velocità MAX
////#define Pin_AnaIbat A1		// Corrente Batteria
////#define Pin_AnaLight A2		// Sensore di luce LDR
							//Potenziometro che regola la velocità MAX
//----------------------------------------------

/*
//////////////////////////////////////////////////////////////////////////
//																		//
//						 M A C R O	  INTERFACCIA PIGPIO 				//
//																		//
//////////////////////////////////////////////////////////////////////////
//
#include <pigpio.h>
#define digitalWrite(p,v) gpioWrite(p, v)
#define OUTPUT PI_OUTPUT
#define INPUT PI_INPUT
//----------------------------------------------
////////////////////////////////////////////////
// ALL GPIO are identified by their Broadcom number.
/////////////////////////////////////////////////

//----------------------------------------------
// pin da configurare in INPUT -----------------

#define Pin_EncRa 23 		/// encoder Right Motor segnale a	
#define Pin_EncLa 24		/// encoder Left Motor segnale a	
#define Pin_irproxy_FW 5	/// sensore di prossimità anteriore
#define Pin_irproxy_FWHL 6	/// sensore di prossimità anteriore per alta luce
#define Pin_irproxy_BK 13	/// sensore di prossimità POSTERIORE
#define Pin_irproxy_FR 19	/// IR Proxy anteriore Destro
#define Pin_irproxy_FL 26	/// IR Proxy anteriore sinistro
//----------------------------------------------
// pin da configurare in INPUT PULLUP	--------
//----------------------------------------------
#define Pin_BumbRight 10 	/// bumper a destra in basso
#define Pin_BumbCenter 9	/// bumper al centro in basso
#define Pin_BumbLeft 11		/// bumper a sinistra in basso
#define Pin_SwitchTop  12	/// deviatore vicino arduino
//----------------------------------------------
// pin da configurare esplicitamente in OUTPUT				----
//----------------------------------------------
#define Pin_MotCK 7		///=FREQUENCYTIMER2_PIN=10	// CLOCK  << vedere in funzione del timer usato Timer 3 =>pin A=5, B=2, C=3
#define Pin_LaserOn 12	// 23 Accensione Laser 1=CW  0=CCW
#define Pin_MotCWR 8	// 23 left CW  - 24 RIGHT MOTOR
#define Pin_MotCWL 20	// 23 RIGHT MOTOR 1=CW  0=CCW
#define Pin_MotENR 25	// 26 RIGHT MOTOR ENABLE
#define Pin_MotENL 16	// 25 LEFT MOTOR ENABLE
#define  Pin_LED_TOP_R 17	/// LED ROSSO
#define  Pin_LED_TOP_G 27	/// VERDE
#define  Pin_LED_TOP_B 22	/// BLU



#define  pinmode(p,v) gpioSetMode(p,v)

///esempi/////////////////////////////////////////////////////////////
///gpioSetMode(17, PI_INPUT);  // Set GPIO17 as input.
///gpioSetMode(18, PI_OUTPUT); // Set GPIO18 as output.
///gpioSetMode(22,PI_ALT0);    // Set GPIO22 to alternative mode 0
///gpioSetPullUpDown(17, PI_PUD_UP);   // Sets a pull-up.
///gpioSetPullUpDown(18, PI_PUD_DOWN); // Sets a pull-down.
///gpioSetPullUpDown(23, PI_PUD_OFF);  // Clear any pull-ups/downs.
/// ///////////////////////////////////////////////////////////////////
*/

//////////////////////////////////////////////////////////////////////////
//																		//
//						 M A C R O	  R O B O T							//
//																		//
//////////////////////////////////////////////////////////////////////////
/// wiring 	Pi		//--------------------------------------------------------------
#define INPUT_PULLUP PUD_UP
/// PiGPio
//#define INPUT_PULLUP PI_PUD_UP
	
//#define COMPASS_CLASS HMC5883L
#define  Pin_LED_TOP Pin_LED_TOP_G
#define Pin_ONBOARD_LED Pin_LED_TOP_B


#define MOTORS_DISABLE 		digitalWrite( Pin_MotENR, !ROBOT_MOTORENABLE_ACTIVE ); digitalWrite(Pin_MotENL, !ROBOT_MOTORENABLE_ACTIVE);
#define MOTORS_ENABLE 		digitalWrite( Pin_MotENR, ROBOT_MOTORENABLE_ACTIVE ); digitalWrite(Pin_MotENL, ROBOT_MOTORENABLE_ACTIVE);

#define LASER_ON digitalWrite(Pin_LaserOn,1);
#define LASER_OFF digitalWrite(Pin_LaserOn,0);
#define RELE1_ON digitalWrite(Pin_Rele1,0);
#define RELE1_OFF digitalWrite(Pin_Rele1,0);
#define WEBCAM_ON digitalWrite(Pin_Rele2,0);
#define WEBCAM_OFF digitalWrite(Pin_Rele2,1);
#define LEDTOP_R_ON digitalWrite(Pin_LED_TOP_R, 1);
#define LEDTOP_ALL_OFF digitalWrite(Pin_LED_TOP_R, 0);digitalWrite(Pin_LED_TOP_G, 0);digitalWrite(Pin_LED_TOP_B, 0);
#define LEDTOP_R_OFF digitalWrite(Pin_LED_TOP_R, 0);
#define LEDTOP_G_ON digitalWrite(Pin_LED_TOP_G, 1);
#define LEDTOP_G_OFF digitalWrite(Pin_LED_TOP_G, 0);
#define LEDTOP_B_ON digitalWrite(Pin_LED_TOP_B, 1);
#define LEDTOP_B_OFF digitalWrite(Pin_LED_TOP_B, 0);
#define TOGGLEPIN(P) digitalWrite(P,!digitalRead(P))

//#define STEPPERLDS_ON digitalWrite(PIN_STEPPERLDS_ENABLE,1);
//#define STEPPERLDS_OFF digitalWrite(PIN_STEPPERLDS_ENABLE,0);



//////////////////////////////////////////////////////////////////////////
//																		//
//	I M P O S T A Z I O N E   P O R T E   R O B O T						//
//																		//
//////////////////////////////////////////////////////////////////////////

inline void initRobotBaseHW() {
	
		

	/// configuro i pin in uscita		(con WiringPi vale la stessa sintassi di arduino)
	pinMode(Pin_MotCK, OUTPUT);			digitalWrite(Pin_MotCK, 1); // il CK è a logica negata : impulso 0 di ampiezza minima di 0.5 uSec. --pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
	pinMode(Pin_MotCWR, OUTPUT);		digitalWrite(Pin_MotCWR, 0);
	pinMode(Pin_MotENR, OUTPUT);		digitalWrite(Pin_MotENR, !ROBOT_MOTORENABLE_ACTIVE);
	pinMode(Pin_MotCWL, OUTPUT);		digitalWrite(Pin_MotCWL, 0);
	pinMode(Pin_MotENL, OUTPUT);		digitalWrite(Pin_MotENL, !ROBOT_MOTORENABLE_ACTIVE);
	pinMode(Pin_LaserOn, OUTPUT);		digitalWrite(Pin_LaserOn, 0);
	pinMode(Pin_LED_TOP_R, OUTPUT);		digitalWrite(Pin_LED_TOP_R, 0);	// led superiore
	pinMode(Pin_LED_TOP_G, OUTPUT);		digitalWrite(Pin_LED_TOP_G, 0);	// led superiore
	pinMode(Pin_LED_TOP_B, OUTPUT);		digitalWrite(Pin_LED_TOP_B, 0);	// led superiore

//	pinMode(Pin_Rele1, OUTPUT);			digitalWrite(Pin_Rele1, 1);
//	pinMode(Pin_Rele2, OUTPUT);			digitalWrite(Pin_Rele2, 1);	// i Rele vanno a logica negata
//	pinMode(Pin_SonarTrig, OUTPUT);		digitalWrite(Pin_SonarTrig, 128);	// pin TRIGGER vs Sonar
//	pinMode(PIN_STEPPERLDS_CW, OUTPUT);	digitalWrite(PIN_STEPPERLDS_CW, 0);	//
//	pinMode(PIN_STEPPERLDS_ENABLE, OUTPUT);	digitalWrite(PIN_STEPPERLDS_ENABLE, 0);	//
//	pinMode(PIN_STEPPERLDS_CK, OUTPUT);	digitalWrite(PIN_STEPPERLDS_CK, 0);	//

	
	
	/// configuro i pin in ingresso (default) ----
	pinMode(Pin_irproxy_FW, INPUT);
	pinMode(Pin_irproxy_FWHL, INPUT);
	pinMode(Pin_irproxy_FR, INPUT);
	pinMode(Pin_irproxy_FL, INPUT);
	pinMode(Pin_irproxy_BK, INPUT);
	
	pinMode(Pin_SwitchTop, INPUT_PULLUP);		//pullUpDnControl (Pin_SwitchTop, INPUT_PULLUP) ;	//Deviatore vs Ground

	pinMode(Pin_BumbRight, INPUT_PULLUP);		//pullUpDnControl (Pin_BumbRight, INPUT_PULLUP) ;
	pinMode(Pin_BumbCenter, INPUT_PULLUP);		//pullUpDnControl (Pin_BumbCenter, INPUT_PULLUP) ;
	pinMode(Pin_BumbLeft, INPUT_PULLUP);		//pullUpDnControl (Pin_BumbLeft, INPUT_PULLUP) ;


	pinMode(Pin_EncRa, INPUT);	///encoder Right Motor 
								//		pinMode(Pin_EncRb,INPUT );	//encoder Right Motor segnale b
	pinMode(Pin_EncLa, INPUT);	///encoder Left Motor
								//		pinMode(Pin_EncLb,INPUT );	//encoder Left Motor segnale b




	//	pinMode(Pin_PIR1, INPUT);	
	//	pinMode(Pin_SonarEcho, INPUT);	// pin ECHO da Sonar
	//-----------------------------------------

	MOTORS_DISABLE;
	LEDTOP_ALL_OFF;
	
	//WEBCAM_OFF;
	//LASER_OFF;
	//RELE1_OFF;


	
}


#endif /* HW_CONFIG_MOTORS_H_ */
