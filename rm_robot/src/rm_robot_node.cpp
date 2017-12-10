/*
tratto da http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
muove il robot sulla base di cmd_vel

subscribe: cmd_vel
publish: info
 */

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>  //replaces #include <lino_msgs/Velocities.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h> //#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>

#include <math.h> /// per fabs()
#include <stdlib.h>

#include <wiringSerial.h>
#include <wiringPi.h>
#include <softTone.h>
#include <pigpio.h>


#include <dbg.h>
#include "parameters.h"
#include "hw_config.hpp"
//~ #include "../include/MyStepperBase.hpp" 


// test thread+++++++++++++++++++++++++++++++++++++++++++
#include "wiringPi.h"
#include "softTone.h"

#define	MAX_PINS	64

#define	PULSE_TIME	100

static int freqs         [MAX_PINS] ;
static pthread_t threads [MAX_PINS] ;

static int newPin = -1 ;


#include <cstdlib>
#include <pthread.h>

using namespace std;

/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///	STEPPER
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////

	#define NUM_CK     1
	pthread_t myMotorThreads[NUM_CK];
	double g_motorCk =0;
	double g_motorCkHalfPeriodUs =0;

	/// thread che genera il clock motori
	void *generateMotorClock(void *threadid) {
		pinMode      (Pin_LED_TOP_B, OUTPUT) ;
		digitalWrite (Pin_LED_TOP_B, LOW) ;
	 
	 
		  struct sched_param param ;

		  param.sched_priority = sched_get_priority_max (SCHED_RR) ;
		  pthread_setschedparam (pthread_self (), SCHED_RR, &param) ;


		
		long tid;
		tid = (long)threadid;
		 
		while (1)
		{

			if ((g_motorCkHalfPeriodUs*2 >  ROBOT_MOTOR_CLOCK_microsecondMIN)
			  &&(g_motorCkHalfPeriodUs*2 <  ROBOT_MOTOR_CLOCK_microsecondMAX))
			{
				digitalWrite(Pin_MotCK,0);
				LEDTOP_B_ON
				delayMicroseconds ( g_motorCkHalfPeriodUs) ;
				LEDTOP_B_OFF
				delayMicroseconds ( g_motorCkHalfPeriodUs) ;
				digitalWrite(Pin_MotCK,1);

			}		
			else
			  delay (1) ;
			
		}

		//pthread_exit(NULL);
		return NULL ;
	}

	/// direzioni del robot
	enum robotDir_e { 
		DIR_S =0 ,	//S=STOP
		DIR_F,	// Forward 
		DIR_B,	//Backward
		DIR_R,	// Right
		DIR_L	// Left 
	};

	/// struttura della classe motore
	typedef unsigned long motCk_t; // was typedef uint32_t motCk; 
	struct StepperDriver_t {
		int pinEnable;
		int pinCW;
		int pinCK;
		
		motCk_t	clockPeriodUs; // clock motori in microsecondi, limitato tra ROBOT_MOTOR_CLOCK_microsecondMIN/MAX
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
		robotDir_e commandDir;
		//position_t poseTarget;	// posizione obiettivo nelle coordinate World
		
		bool blSpeedLimiterOn ;

	};

	/// classe motore
	class MyStepperBase_c
	{
		public:
			StepperDriver_t cmd;  //contiene commandDir
			MyStepperBase_c();
			void init();
			void goCmdVel(float targetVelocityLinear, float targetVelocityAngular);
			void stop();
			void enable();
			
			robotDir_e getCmdDir();
			char getCmdDirStr();
			
		private:
		//	robotDir_e _commandDir;
			//~ void _goHz(robotDir_e dir, float stepsPerSecond);
			//~ int _motorClockLimiter(int ckInterval); // limita il clock tra i due limiti ROBOT_MOTOR_CLOCK_microsecondMAX  e ROBOT_MOTOR_CLOCK_microsecondMIN
			void _motorSetPWMCK(float herz);
			void _setMotorDir(robotDir_e dir);



	};

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
		cmd.blSpeedLimiterOn = true;
	}
	/// Converte la frequenza in periodo del clock


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
	 
	 
	/// converte steps per second in periodo di clock, usato dal thread che genera la frequenza
	void MyStepperBase_c::_motorSetPWMCK(float stepPerSecond){

		if (stepPerSecond > 0.0)
		{
			/// controlla che il clock sia entro i limiti
			if (cmd.blSpeedLimiterOn)
			{
				if (stepPerSecond > ROBOT_MOTOR_STEPSPERSECOND_MAX) {
					stepPerSecond = ROBOT_MOTOR_STEPSPERSECOND_MAX;
				}
				else if (stepPerSecond < ROBOT_MOTOR_STEPSPERSECOND_MIN) {
					stepPerSecond = ROBOT_MOTOR_STEPSPERSECOND_MIN;
				}
			}
			
			/// interfaccia col thread
			g_motorCk =stepPerSecond;
			g_motorCkHalfPeriodUs =500000 /g_motorCk;
			/// interfaccia WiringPi: 
			softToneCreate(Pin_MotCK);
			softToneWrite (Pin_MotCK, g_motorCk) ; /// wiringpi
			
		}else
		{
			g_motorCk = 0;
			g_motorCkHalfPeriodUs = -1;
			softToneCreate(Pin_MotCK);
			softToneWrite (Pin_MotCK, g_motorCk) ; /// wiringpi
		}

	}
	/// ritorna l'ultima direzione impostata
	robotDir_e MyStepperBase_c::getCmdDir() {
		return cmd.commandDir;
	}
	/// ritorna la direzione corrente in formato stringa
	char MyStepperBase_c::getCmdDirStr() {

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

	/// imposta solamente status.cmd.cwL e status.cmd.cwR
	void MyStepperBase_c::_setMotorDir(robotDir_e dir) {
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

	/// abilita i motori 
	void MyStepperBase_c::enable() {
		MOTORS_ENABLE	
	}

	/// ///////////////////////////////////////////////////
	/// Ferma i motori disabilitando gli avvolgimenti
	void MyStepperBase_c::stop() {
		cmd.enL = 0;
		cmd.enR = 0;

		MOTORS_DISABLE;
		_motorSetPWMCK(0.0);
		cmd.isMoving = false;	// segnala che si sta muovendo
		cmd.commandDir = DIR_S;

	}

	/// /////////////////////////////////////////////////////////////////
	/// interfaccia ROS
	/// memorizza le velocità richieste
	/// in cmd.targetVelocityLinear e ..Angular
	/// in base ai valori di velocità, imposta cmdDir
	/// /////////////////////////////////////////////////////////////////
	void MyStepperBase_c::goCmdVel(float targetVelocityLinear, float targetVelocityAngular) 
	{
		float stepsperseconds = 0.0;
		float ck;

		cmd.targetVelocityLinear = targetVelocityLinear;
		cmd.targetVelocityAngular = targetVelocityAngular;
		dbg2("\n targetVelocityLinear: %f",cmd.targetVelocityLinear);	
		dbg2(" targetVelocityAngular: %f \n",cmd.targetVelocityAngular);

		if (abs(cmd.targetVelocityLinear) >  MOTOR_MINIMUM_LINEAR_VELOCITY)
		{
			
			/// converto da velocità lineare a  step per secondo
			stepsperseconds= fabs(cmd.targetVelocityLinear)*ROBOT_M2STEPS;	///ROBOT_M2STEPS 3536.7765	
			_motorSetPWMCK(stepsperseconds); 
			
			dbg2("\n\t stepsperseconds LIN \t %f \n",stepsperseconds)
					
			if (cmd.targetVelocityLinear > 0)
			{
				/// FORWARD			
				_setMotorDir(DIR_F);				
				//MSG("F");
			}
			else
			{
				///BACK			
				_setMotorDir(DIR_B);
				//MSG("B");
			}
			cmd.isMoving = true;	/// segnala che si sta muovendo
			enable();

		}
		else if (fabs( cmd.targetVelocityAngular) > MOTOR_MINIMUM_ANGULAR_VELOCITY)  /// comando di ANGULAR VELOCITY
		{


			/// converto da velocità angolare a step per secondo
			stepsperseconds= fabs(cmd.targetVelocityAngular)*ROBOT_RAD2STEPS; 	///ROBOT_RAD2STEPS=604.789
			_motorSetPWMCK(stepsperseconds);
			dbg2("\n\t stepsperseconds ANG %f\t",stepsperseconds)
			
			
			if (cmd.targetVelocityAngular > 0)
			{
				//ccw
				_setMotorDir(DIR_L);
				
			}
			else
			{
				//cw
				_setMotorDir(DIR_R);			

			}
			cmd.isMoving = true;	/// segnala che si sta muovendo
			enable();

		}
		else
		{
			g_motorCk = 0;
			g_motorCkHalfPeriodUs = -1; ///stop generazione clock
			stop();

		}
		
	//	dbg2(" dir: %s ", getCmdDirStr());
		dbg2("\n\t\t g_motorCkHalfPeriodUs: %d  \n", g_motorCkHalfPeriodUs);

	}

	MyStepperBase_c robotMotors;

/// ///////////////////////////////////////////////////////////////////
///	END STEPPER
/// ///////////////////////////////////////////////////////////////////
/*
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
/// ENCODERS 
/// opera solo se  ogni ISR_MINMUM_INTERVAL_MSEC
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
	#include <sys/time.h>
	#include <wiringPi.h>

	// Which GPIO pin we're using
	#define PIN 2
	// How much time a change must be since the last in order to count as a change
	#define IGNORE_CHANGE_BELOW_USEC 10000
	struct encoder_t{
		int count;  // contatore encoder 
		bool interruptFlag ;
		struct timeval last_change;
	};
	encoder_t encR, encL;

	/// aggiorna i contatori di encL e encR	  
	#define ISR_MINMUM_INTERVAL_MSEC 15  // 30 = circa ROBOT_MOTOR_STEPS_PER_ENCODERTICK * ROBOT_MOTOR_STEPS_PER_ENCODERTICK


	/// Interrupt service routines for the right motor's  encoder
	void ISRencoderR()
	{
		struct timeval now;
		gettimeofday(&now, NULL);//isrCurrCallmSec = millis();

		// Time difference in usec
		unsigned long diff;
		diff = (now.tv_sec * 1000000 + now.tv_usec) - (encR.last_change.tv_sec * 1000000 + encR.last_change.tv_usec);
		
		
		TOGGLEPIN(Pin_LED_TOP_B);
		
		// Filter jitter
		if (diff > IGNORE_CHANGE_BELOW_USEC) //		if (isrCurrCallmSec > (isrLastCallmSec + ISR_MINMUM_INTERVAL_MSEC))
		{
			gettimeofday(&encR.last_change, NULL);		//encR.last_change = now; //isrLastCallmSec = isrCurrCallmSec;				
			
			/// incremento o decremento il contatore in base al movimento del robot
			// to do : o se è fermo in base al segno dell'accelerazione'	
			switch (robotMotors.getCmdDir())
			{
				case DIR_F: 
					encR.count +=1 ; //robot.status.sensors.encR += 1;  				
					break;
				case DIR_B :
					encR.count -=1 ; //robot.status.sensors.encR += 1;  
					break;
				case DIR_L :
					encR.count +=1 ; //robot.status.sensors.encR += 1;  
					break;
				case DIR_R :
					encR.count -=1 ; //robot.status.sensors.encR += 1;  
					break;
					
				default:
					break;
					
			}
				
		}
	}

	/// Interrupt service routines for the left motor's  encoder
	void ISRencoderL()
	{
		struct timeval now;
		gettimeofday(&now, NULL);//isrCurrCallmSec = millis();

		// Time difference in usec
		unsigned long diff;
		diff = (now.tv_sec * 1000000 + now.tv_usec) - (encR.last_change.tv_sec * 1000000 + encR.last_change.tv_usec);

		TOGGLEPIN(Pin_LED_TOP_R);
		
		// Filter jitter
		if (diff > IGNORE_CHANGE_BELOW_USEC) //		if (isrCurrCallmSec > (isrLastCallmSec + ISR_MINMUM_INTERVAL_MSEC))
		{
			gettimeofday(&encL.last_change, NULL); //encR.last_change = now;//isrLastCallmSec = isrCurrCallmSec;				


			/// incremento il contatore solo se mi sto muovendo
			switch (robotMotors.getCmdDir())
			{
				case DIR_F :
					encL.count +=1 ; //robot.status.sensors.encR += 1;  				
					break;
				case DIR_B :
					encL.count -=1 ; //robot.status.sensors.encR += 1;  
					break;
				case DIR_L :
					encL.count -=1 ; //robot.status.sensors.encR += 1;  
					break;
				case DIR_R :
					encL.count +=1 ; //robot.status.sensors.encR += 1;  
					break;
					
				default:
					break;					
			}
			
		}
	}
	
	/// azzero i contatori e i flag di interrupt
	void encoder_reset()
	{
		encR.count=0;
		encL.count=0;
		encR.interruptFlag=false;
		encL.interruptFlag=false;
	
	}	

	/// inizializzo i contatori le interrupt service routine dell'encoder
	void encoder_setup()
	{
		// Bind to interrupt
		wiringPiISR(Pin_EncRa, INT_EDGE_BOTH, &ISRencoderR);
		wiringPiISR(Pin_EncLa, INT_EDGE_BOTH, &ISRencoderL);

		encoder_reset();	
		dbg("\nEncoders setup");
	}


/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
/// SENSORI 
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////

	bool isSensorPirOn(){

		return digitalRead(Pin_irproxy_FW)	;
	}

*/



/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///	R O S
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////

	//ros::NodeHandle nh; deve essere dichiarato dopo ros:::Init()

	double rate = 10.0; // 5Hz main loop rate



	double g_vel_x = 0.0;
	double g_vel_y = 0.0;	/// sempre 0
	double g_vel_z = 0.0; ///rotazione




	double g_vel_dt = 0.0;
	double g_imu_dt = 0.0;
	double g_imu_z = 0.0;

	ros::Time g_last_loop_time(0.0);
	ros::Time g_last_vel_time(0.0);
	ros::Time g_last_imu_time(0.0);



	void cbk_cmdVel( const geometry_msgs::Twist& vel) {
		//callback every time the robot's linear velocity is received
		ros::Time current_time = ros::Time::now();

		dbg2("\n received cmd_vel lin:%f , rot: %f \n", vel.linear.x, vel.angular.z);
		
		robotMotors.goCmdVel(vel.linear.x, vel.angular.z) ;	 
		ROS_INFO("received cmd_vel lin:%f , rot: %f", vel.linear.x, vel.angular.z);
		
		// todo : sostituire con i dati imu
		g_vel_x = vel.linear.x;
		g_vel_y = vel.linear.y;
		g_vel_z = vel.angular.z;

		g_vel_dt = (current_time - g_last_vel_time).toSec();
		g_last_vel_time = current_time;
		
		//~ dbg2("cmd_vel.x: %f",g_vel_x);
		//~ dbg2("cmd_vel.z: %f",g_vel_z);

	}

	void cbk_IMU( const sensor_msgs::Imu& imu){
		//callback every time the robot's angular velocity is received
		ros::Time current_time = ros::Time::now();
		//this block is to filter out imu noise
		if(imu.angular_velocity.z > -0.03 && imu.angular_velocity.z < 0.03)
		{
			g_imu_z = 0.00;
		}
		else
		{
			g_imu_z = imu.angular_velocity.z;
		}

		g_imu_dt = (current_time - g_last_imu_time).toSec();
		g_last_imu_time = current_time;
	}




/* 
 * spostato in rm_sensors
 
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///	Robot ODOMETRY
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
	// Current state of the pin
	static volatile int state;
	// Time of last change
	struct odom_t{
		float x;
		float y;
		float r;
		struct timeval last_change;
	};
	
	/// Variabile globale che tiene l'odometria del robot'
	odom_t robotOdom;

	/// resetta  odom 
	void odom_reset(){

		robotOdom.r=0.0;
		robotOdom.x=0.0;
		robotOdom.y=0.0;
		gettimeofday(&robotOdom.last_change ,NULL);
		
		gettimeofday(&encR.last_change, NULL);

		gettimeofday(&encL.last_change, NULL);
	 
	}
	

	/// utilizzando i contatori degli encoder aggiorna odom e azzera i contatori
	/// in: encL, encR count
	/// out: robotOdom
	void odom_update(int encLcount, int encRcount){
		/// calcolo basato sulle velocità che però io non utilizzo
		///calculate angular displacement  θ = ω * t
		//double delta_x = (linear_velocity_x * cos(theta) - linear_velocity_y * sin(theta)) * g_vel_dt; //m
		//double delta_y = (linear_velocity_x * sin(theta) + linear_velocity_y * cos(theta)) * g_vel_dt; //m
		//double delta_theta = angular_velocity * g_imu_dt; //radians
		
		if ( abs(encLcount)+abs(encRcount) > 0) ///somma 0 se ruota
		{
			dbg2("\nEncL.count: %f, R: %f ",encLcount,encRcount);
			
			
			/// delta rotazione è dato dalla differenza tra i due encoder trasformata in step e poi in radianti		
			double delta_theta = (float)(encRcount- encLcount )*(float)ROBOT_MOTOR_STEPS_PER_ENCODERTICK /(float)ROBOT_MOTOR_STEPS_PER_RADIANT;
			double delta_x = (float)((encL.count+encR.count)/2)*(float)ROBOT_MOTOR_STEPS_PER_ENCODERTICK /(float)ROBOT_MOTOR_STEPS_PER_M; ///somma 0 se ruota
			double delta_y = sin(delta_theta)* ROBOT_WEEL_DISTANCE/2;
			
			dbg2("\nDelta X: %f, y:%f , r:%f \n",delta_x,delta_y,delta_theta);
			
			
			///calculate current position of the robot
			robotOdom.x += delta_x;
			robotOdom.y += delta_y; 
			robotOdom.r += delta_theta; 
			gettimeofday(&robotOdom.last_change ,NULL);
			
			dbg2("\nOdom.x: %f, y:%f , r: %f \n",robotOdom.x, robotOdom.y, robotOdom.r);
			

			/// azzero i contatori
			encoder_reset();
			
		}
				
	}

/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///  ROS odom                                 
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
	#pragma region ROS odom

		#include <tf/tf.h>		// richiesto da  createQuaternionFromYaw

		#include <nav_msgs/Odometry.h>
		unsigned long odom_time;

		nav_msgs::Odometry rosmsg_odom;
		
		//ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 50);//ros::Publisher pub_odom("/odom", &rosmsg_odom);
		//ros::Publisher pub_odom ;

		/// ////////////////////////////////////////////////////////////////////
		/// Pubblica  odometry 
		/// ////////////////////////////////////////////////////////////////////
		/// subito se publishNow=true, oppure secondo la schedulazione prevista	
		void publish_odom(ros::Publisher* pub_odom, bool publishNow=false) {
			if (publishNow ||(millis() > odom_time)) {
			
			
		
				//nav_msgs::Odometry odom;
				rosmsg_odom.header.stamp = ros::Time::now();
				rosmsg_odom.header.frame_id = "odom";
				rosmsg_odom.child_frame_id = "base_link";

				///set the position
				rosmsg_odom.pose.pose.position.x = robotOdom.x;
				rosmsg_odom.pose.pose.position.y = robotOdom.y;
				rosmsg_odom.pose.pose.position.z = 0.0;


				///robot's heading in quaternion
				//~ tf2::Quaternion q;
				//~ q.setRPY(0, 0, robotOdom.r);


				rosmsg_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotOdom.r);

				/// set the velocity
				rosmsg_odom.twist.twist.linear.x = robotMotors.cmd.targetVelocityLinear;  //impostato da robotMotors.goCmdVel
				rosmsg_odom.twist.twist.linear.y = 0.0;
				rosmsg_odom.twist.twist.angular.z = robotMotors.cmd.targetVelocityAngular;
				
				/// angular speed from IMU
				rosmsg_odom.twist.twist.angular.x = 0.0;
				rosmsg_odom.twist.twist.angular.y = 0.0;				
				rosmsg_odom.twist.twist.angular.z =g_vel_z; //todo: da g_imu_z;
				
				
				//TODO: include covariance matrix here

				//publish the message
				pub_odom->publish(rosmsg_odom);
				
				//ROS_INFO("pub odom");
				odom_time = millis() + 500;
			}
		}

	#pragma endregion
	/// inzializza  odom 
	void odom_setup(){
		odom_reset();
			 
	}
/// end odometry ///////////////////////////////////////////////////////

	void publish_tf(odom_t *odom){

	  static tf2_ros::TransformBroadcaster tfbr;
	  geometry_msgs::TransformStamped transformStamped;
	  
	  transformStamped.header.stamp = ros::Time::now();
	  transformStamped.header.frame_id = "odom";
	  transformStamped.child_frame_id = "base_link";
	  
	  
	  
	  transformStamped.transform.translation.x = odom->x;
	  transformStamped.transform.translation.y = odom->y;
	  transformStamped.transform.translation.z = 0.0;
		transformStamped.setOrigin( tf::Vector3(odom->x, odom->y, 0.0) );
	  
	  tf2::Quaternion q;
	  q.setRPY(0, 0, odom->r);
	  transformStamped.transform.rotation.x = q.x();
	  transformStamped.transform.rotation.y = q.y();
	  transformStamped.transform.rotation.z = q.z();
	  transformStamped.transform.rotation.w = q.w();

	  tfbr.sendTransform(transformStamped);


	}

/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///  ROS check PIR                                 
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
#define INTERVAL_GETSENSORS_SEC 10
ros::Time g_lastsensor_time;
#include "std_msgs/String.h"		/// richiesto da std_msgs::String msg;

void	checkSensors(ros::Publisher* pub){
		ros::Time now = ros::Time::now();
		double dt = (now - g_lastsensor_time).toSec();
		if (dt > INTERVAL_GETSENSORS_SEC){

			///-----------------------------------------------------
			if (isSensorPirOn())
			{
				ROS_INFO("HUMAN DETECTED");	
				
				std_msgs::String msg;
				msg.data = "Ciao chi sei ?";
				pub->publish(msg);
			}
			
			///------------------------------------------------------
			g_lastsensor_time = now;
		}
}
*/



/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///	GLOBAL FUNCTIONS
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
	void HWtest(){

		dbg2("\nStato di Pin_irproxy_FW: %d \n" , digitalRead(Pin_irproxy_FW));
		
		/// per fare 90° (Pi/2 = 1.571 rad) 
		/// devo attivare attendere ( s= t*v) >> t=s/v  = 1.571 /0.5= 3142ms
		
		/// CCW
		robotMotors.goCmdVel(0.0,0.5);
		delay(3142);
		/// CW
		robotMotors.goCmdVel(0.0,-0.5);
		delay(3142);
		/// STOP
		robotMotors.goCmdVel(0.0,-0.0);		
	}
	
	
	void HWinit(){
		wiringPiSetup();
		
		
		//~ if (gpioInitialise() < 0)
		//~ {
		 //~ std::cout << "pigpio initialisation failed."<< std::endl;  // pigpio initialisation failed.
		 //~ std::cout << "prova a fare: sudo killall pigpiod  , e pio riprova"<< std::endl;
		//~ }	
		
		//~ initRobotBaseHW();
		
		/// MOTORI ////////////////////////////////////////////////////
		pinMode(Pin_MotCK, OUTPUT);			digitalWrite(Pin_MotCK, 1); // il CK è a logica negata : impulso 0 di ampiezza minima di 0.5 uSec. --pinMode(FREQUENCYTIMER2_PIN, OUTPUT);
		pinMode(Pin_MotCWR, OUTPUT);		digitalWrite(Pin_MotCWR, 0);
		pinMode(Pin_MotENR, OUTPUT);		digitalWrite(Pin_MotENR, !ROBOT_MOTORENABLE_ACTIVE);
		pinMode(Pin_MotCWL, OUTPUT);		digitalWrite(Pin_MotCWL, 0);
		pinMode(Pin_MotENL, OUTPUT);		digitalWrite(Pin_MotENL, !ROBOT_MOTORENABLE_ACTIVE);
		
		/// ENCODER ///////////////////////////////////////////////////
//		pinMode(Pin_EncRa, INPUT);	///encoder Right Motor 
//		pinMode(Pin_EncLa, INPUT);	///encoder Left Motor

		/// LED ///////////////////////////////////////////////////////
		pinMode(Pin_LED_TOP_R, OUTPUT);		digitalWrite(Pin_LED_TOP_R, 0);	// led superiore
		pinMode(Pin_LED_TOP_G, OUTPUT);		digitalWrite(Pin_LED_TOP_G, 0);	// led superiore
		pinMode(Pin_LED_TOP_B, OUTPUT);		digitalWrite(Pin_LED_TOP_B, 0);	// led superiore
		
		//~ //robotMotors.stop();
		LEDTOP_R_ON	// indica di aspettare SETUP  in corso
		
		
		/// CLOCK MOTORI  ///////////////////////////////////////		
		//+++++++++++++++++++++++++++++++++++++++++++
		softToneCreate (Pin_MotCK);
		//softToneCreate (Pin_LaserOn);
		//softToneWrite (Pin_LaserOn, 5); //test laser
		
		int i =0; /// indice thread
		///creo thread di generazione clock motori
		//~ int  rc = pthread_create(&myMotorThreads[i], NULL, generateMotorClock, (void *)i);
		//+++++++++++++++++++++++++++++++++++++++++++

		//encoder_setup();/// attiva ISR

		HWtest();
		
	}

	void HWshutdown(int i){
		/// stop degli altri threads
		pthread_cancel (myMotorThreads[i]) ;
		pthread_join   (myMotorThreads[i], NULL) ;
		myMotorThreads[i] = 0 ;
		LEDTOP_ALL_OFF ;

	}


	/// pubblica regolarmente alcuni dati -------------------------------------------
	#define INTERVAL_INFO_POS_SEC 5
	ros::Time g_last_info_time;
	
	void publish_info(ros::Time current_time){
		/// Invio a bassa frequenza info
		double dt = (current_time - g_last_info_time).toSec();
		if (dt > INTERVAL_INFO_POS_SEC){
		
			///---------------------------------------------------------
			//		ROS_INFO("robotOdom.x: [%f], y: [%f], r: [%f]",robotOdom.x,robotOdom.y ,robotOdom.r);
			//		ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg_odom->pose.pose.orientation.x, msg_odom->pose.pose.orientation.y, msg_odom->pose.pose.orientation.z, msg_odom->pose.pose.orientation.w);
					ROS_INFO("Vel-> Linear: [%f], Angular: [%f]",g_vel_x,g_vel_z);
			
	
			
			///---------------------------------------------------------
			g_last_info_time = current_time;
		}
	}

/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
///	M A I N
/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////
int main(int argc, char** argv){

	
	///---------------------------------------------------------------
	/// HW inizialization 
	///---------------------------------------------------------------
	HWinit();

	///---------------------------------------------------------------
	
	///---------------------------------------------------------------
	/// ROS inizialization 
	///---------------------------------------------------------------
	ros::init(argc, argv, "rm_robot_node"); ///must be called before creating a node handle
	
	ros::NodeHandle nh; 


	ros::Subscriber sub_cmd_vel = nh.subscribe("/cmd_vel", 50, cbk_cmdVel);
	ros::Subscriber imu_sub = nh.subscribe("/imu", 50, cbk_IMU);
	
// 	odom_setup();
//	ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/odom", 50);//ros::Publisher pub_odom("/odom", &rosmsg_odom);
// 	ros::Publisher pub_chatter = nh.advertise<std_msgs::String>("/chatter", 1000);


	///---------------------------------------------------------------
	/// ROS parameters 
	///---------------------------------------------------------------
	//void:ros::param::set(parameter_name, input_value); /// to set a parameter use
	//void:ros::param::get(parameter_name); ///to retrieve a parameter value from the parameter server
	
	/// Setting a parameter value during a launch file 
	/// <param name="param-name" value="param-value" />
	
	LEDTOP_R_OFF
	LEDTOP_G_ON  // segnale READY


	ros::Rate r(rate);
	
	
	while(nh.ok()){
		ros::spinOnce();
		ros::Time current_loop_time = ros::Time::now();

		///linear velocity is the linear velocity published from the Teensy board in x axis
		//double linear_velocity_x = g_vel_x;

		///linear velocity is the linear velocity published from the Teensy board in y axis
		//double linear_velocity_y = g_vel_y;

		///angular velocity is the rotation in Z from imu_filter_madgwick's output
		//double angular_velocity = g_imu_z;



		/// devo aggiornare la posizione con regolarità e almeno prima di eseguire un /cmd_vel
		//odom_update(encL.count,encR.count);


		/// pubblica robotOdom
		//publish_odom(&pub_odom);//odom_pub.publish(odom);
		
		/// tf lo dovrebbe calcolare efk
		//publish_tf(&robotOdom);
		
		/// lettura sensori
		//checkSensors(&pub_chatter);
		
		
		publish_info(current_loop_time);

		g_last_loop_time = current_loop_time;
		r.sleep();
		
	}
	HWshutdown(0);

}
