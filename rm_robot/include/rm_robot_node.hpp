/////////////////////////////////////////////////////////////////////////////////////////////
// CONTIENE LE STESSE FUNZIONI DI ROBOT.CPP MA I COMANDI LI INVIA TRAMITE SERIALE AL CORE  //
/////////////////////////////////////////////////////////////////////////////////////////////

#if !defined(__ROBOTMODELSMALL_H__)
#define __ROBOTMODELSMALL_H__


//#define dbg2(msg, v)   Serial.print(F(msg)); Serial.println(v);
//#define dbg(msg)  Serial.println(F(msg));
//#define dbg(fmt, ...) (#if DEBUG_ENABLED (Debug.log(fmt, ## __VA_ARGS__); #endif)

////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////
//// INCLUDE
////////////////////////////////////////////////////////////////////////////




/// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt16.h>

/// Boost includes
#include <boost/thread/mutex.hpp>

/// Kobuki includes
//#include <kobuki_msgs/BumperEvent.h>

/// RPNavigation includes
//#include <direction_source.h>




#include <hw_config.hpp>
#include <msgCore.h>
#include <parameters.h>	// le costanti sono state spostate qui
#include <MyStepperBase.hpp>

/*


	typedef unsigned long motCk_t; // was typedef uint32_t motCk; 
	struct position_t {
		double x; // posizione verso est in mm ripetto a Home
		double y; // posizione a nord in mm ripetto a Home
		double r;	//angolo di rotazione rispetto al nord in radianti // non più gradi 
	};
	struct irproxy_t {
		unsigned fw : 1;	//Forward
		unsigned fwHL : 1;	//Forward High Light
		unsigned fr : 1; //Forward right
		unsigned fl : 1;	//Forward left
		unsigned bk : 1; //back

	};
	struct bumper_t {
		unsigned right : 1;	//Forward
		unsigned center : 1;	//Forward High Light
		unsigned left : 1;	//Forward left
	};
	struct gps_t {
		int sats;
		double  lat;
		double  lng;
		double alt;
		int homeDistCm;
		int homeAngleDeg;
	};
	struct sensors_t {
		unsigned long ts; // sensor data timestamp in milliseconds
		irproxy_t	irproxy;
		bumper_t bumper;
		int sonarEchos[SONAR_ARRAY_SIZE];	//distanza rilevata in funzione dell'angolo max 180
		int ldsEchos_cm[360];	//distanza in cm rilevata in funzione dell'angolo 0-359
		int light;
		bool switchTop;
		bool pirDome;
		bool pir2;
		bool ignoreIR; // impostare a true per non usare i sensori di prossimità agli infrarossi 

		bool encL;	// stato encoder Left
		bool encR;	// stato encoder RIGHT};
		bool encLprec;	// vecchio valore contatore  posizione encoder Left
		bool encRprec;	// vecchio valore  contatore  posizione encoder RIGHT};
		unsigned int  encRcnt;	// contatore  posizione encoder RIGHT};
		unsigned int  encLcnt;	// contatore  posizione encoder RIGHT};
		long	analog[5]; //0..1023
		unsigned int batCharge;	// livello batteria 0-100%
		gps_t gps;
		//#if OPT_ENCODERS
		//
		//	Encoder_state_t  encL;		// statico perchè deve essere visibile all'ISR?
		//	Encoder_state_t  encR;

		//#endif
	};
	struct dim_t {	///dimensioni fisiche
		float BodyRadius;
		float WeelRadius;
	};
	struct parameter_t {
		//		long stepsPerCm	;
		int maxspeed_ck;	//clock col valore piu basso
		int minspeed_ck;	//clock col valore piu alto
		int sonarScanSpeed;
		int sonarMaxDistance;	// distanza massima SONAR in cm = MAX_DISTANCE
		int sonarEndAngle;		// ampiezza scansione da 1 a 180
		int sonarStepAngle;		// risoluzione es 5
		int sonarStartAngle;	// angolo iniziale in gradi pos. ccw 0= davanti
		int sonarScanSweeps;	// numero di passate
		int sonarScanSteps;		// step = sonarScanAngle/sonarStepAngle// contatore del numero di echo da inviare
		int sonarMedianSamples;		// 
		int SonarMaxDistAngle;		// angolo in cui nella scansione trova la distanza massima
		int sonarMinDistance; //in mm
	};
	struct actuators_t {
		bool laserOn;
		bool MotENR;
		bool MotENL;
		bool blueToothOn;
		unsigned int rele[10];

	};
	/// Flag degli eventi da segnalare
	struct pendingEvents_t {
		bool EventFlag; // true quando almenu un evento è stato sollevato
		bool operatingMode;//segnala un cambio nel modo operativo
		bool irproxyF;//segnala un cambio nei sensori di prossimità
		bool irproxyFH;//segnala un cambio nei sensori di prossimità
		bool irproxyB;//segnala un cambio nei sensori di prossimità
		bool irproxyR;//segnala un cambio nei sensori di prossimità
		bool irproxyL;//segnala un cambio nei sensori di prossimità
		bool bumperF;
		bool bumperR;
		bool bumperL;
		bool light;//segnala un cambio nel'intensità di luce
		bool switchTop;//segnala un cambio nello switch top
		bool pirDome;//segnala un cambio nel sensore di movimento
		bool pir2;
		bool encL;	//segnala un cambio nell' encoder Left
		bool encR;	//segnala un cambio nell' encoder RIGHT};
		bool analog[5]; //segnala un cambio negli ingressi analogici
		bool batCharge;	// segnala un cambio del livello batteria 0-100%
		bool gps;	// segnala un aggiornamento nel GPS
		bool posCurrent; //segnala un cambio nella posizione corrente
		bool rele0;	//segnala un cambio nel rele 0
		bool rele1;	//segnala un cambio nel rele 1
		unsigned long	ts;	//timestamp dell'evento piu recente
	};
	/// Stato del Robot
	struct robotStatus_t {
		bool	tictac;	// periodicamente switchato onoff

		sensors_t		sensors;
		actuators_t		act;
		parameter_t		parameters;	// parametri operativi
		//cmd_t			cmd;		// spostato in MyStepperBase.h
		position_t		posCurrent; //posizione corrente in [mm] e [rad]
		pendingEvents_t pendingEvents;
		bool			isMoving;
		unsigned long	ts;	//timestamp
	};
	/// Odometry
	struct odom_t {
		float x;
		float y;
		float th;
	};
	enum robotSpeed_e  //FREQUENZE DI CLOCK CORRISPONDENTI 
	{
		MIN = ROBOT_MOTOR_CLOCK_microsecondMAX,
		SLOW = 4000,
		MEDIUM = 3500,
		FAST = 3000,
		MAX = ROBOT_MOTOR_CLOCK_microsecondMIN
	};


*/








class RobotNode
{
    private:
		/** copiati da locomotion di Robot Photographer */
        ros::NodeHandle& node;                      /**< Node's handle.                         */
		ros::Subscriber sub_cmd_vel;
		
    public:
        /**
         * Default node's constructor.
         * @param node Handle to ROS node.
         */
        RobotNode(ros::NodeHandle& node);	////MyRobotModelSmall_c();	


		//////////////////////////////////////////////////////////////////////////
		/// Proprietà Pubbliche definite da me
		//////////////////////////////////////////////////////////////////////////

		void callback_cmd_vel( const geometry_msgs::Twist& cmd_vel); 
		
		//void initRobotBaseModel();
		//MyStepperBase_c mot;	// richiede #include <MyRobotLibs\MyStepperBase\MyStepperBase.h>
};



#endif 






