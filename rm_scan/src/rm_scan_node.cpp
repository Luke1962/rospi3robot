//--------------------------------------------------------------------
// Genera messaggio ros /scan partendo dall'immagine della webcam 
//--------------------------------------------------------------------
/// eseguire prima il comando seguente per attivare il laser a 4Hz
/// pigs hp 18 4 500000 
/// L'accensione interferisce con l'audio out
//////////////////////////////////////////////////////////////////////
//																	//
//	ROS includes and node handler									//
//																	//
//////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>



// OpenCV includes ----------------------
//#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"


#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>



#include <iostream>
#include <wiringPi.h>

#include <raspicam/raspicam_cv.h>

// #define USE_PIGPIO
#ifdef USE_PIGPIO
	#include <pigpio.h>
#else
	#include <softTone.h>
#endif



#include <sstream>





using namespace cv;
using namespace std;  //per usare cout anzicè std::cout


//////////////////////////////////////////////////////////////////////
//																	//
//	UTILITIES														//
//																	//
//////////////////////////////////////////////////////////////////////

#include <stdarg.h>  // For va_start, etc.
/// Ritorna una stringa formattata 
std::string string_format(const std::string fmt, ...) {
    int size = ((int)fmt.size()) * 2 + 50;   // Use a rubric appropriate for your code
    std::string str;
    va_list ap;
    while (1) {     // Maximum two passes on a POSIX system...
        str.resize(size);
        va_start(ap, fmt);
        int n = vsnprintf((char *)str.data(), size, fmt.c_str(), ap);
        va_end(ap);
        if (n > -1 && n < size) {  // Everything worked
            str.resize(n);
            return str;
        }
        if (n > -1)  // Needed size returned
            size = n + 1;   // For null char
        else
            size *= 2;      // Guess at a larger size (OS specific)
    }
    return str;
}


//////////////////////////////////////////////////////////////////////
//																	//
//	GESTIONE LASER													//
//																	//
//////////////////////////////////////////////////////////////////////
	#if USE_PIGPIO
		/// non devo fare nulla
		#define PIN_LASER 29 // stessa numerazione di GPIO   29 = :::::::::.....°
		
	#else /// uso  interfaccia WiringPi: 
		#define PIN_LASER 1 // stessa numerazione di GPIO   29 = :::::::::.....°
		
	#endif
int LASER_ON_DELAY =0	; //80
//abbastanza  stabile imgOn a 265ms, 77mm (oscillazioni nella luminosità)
// con 500ms ok (minime variazioni di intensità)
int LASER_OFF_DELAY =0 ;// con 500ms si vede ancora il laser, con 800  e 1000 si e no
// con delay On a 77ms è necessario portare delayOff a 80ms per ottenere una differenza con variazioni minime
int LaserFreq = 4;


void setLaserFreq(int Hz){
	if (	Hz < 0)
	{
		Hz =0;
	}
	#if USE_PIGPIO
		std::string strTmp;
		strTmp = string_format("sudo pigpiod ; pigs hp 18 %d 500000; ",Hz);
		const char *sysCmd = strTmp.c_str();
		system(sysCmd);
	#else //uso WiringPi
		softToneWrite (PIN_LASER, Hz); //test laser
	#endif
}
void initLaser(){
	pinMode (PIN_LASER, OUTPUT) ;// il 25 corrisponde a GPIO 26

	#if USE_PIGPIO
		/// non devo fare nulla
		
	#else /// uso  interfaccia WiringPi: 
		softToneCreate(PIN_LASER);
	#endif


	setLaserFreq(LaserFreq);
	
}
void laserOn(int delayAfter=0){
	digitalWrite (PIN_LASER, HIGH); 			//gpio26->setval_gpio("1"); // turn  ON
	delay(delayAfter); /// millisec
}
void laserOff(int delayAfter=0){
	digitalWrite (PIN_LASER,  LOW) ;
 	delay(delayAfter);
 	
}
void testLaser(){
  for (int i=0;i<5;i++)
  {
    laserOn(300);
    laserOff(300);
  }
}

void OnChangeLaserFrequency(int, void*){
	setLaserFreq(LaserFreq);
	//ROS_INFO(sysCmd);
}

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
//																	//
//	Global variables												//
//																	//
//////////////////////////////////////////////////////////////////////
/// 

int mousPosX = 100;
int mousePosY=100;
int crossHalfSize = 25;
int cameraExposure=0;
int cameraGain=0;
int debugOn =1;


Mat src, erosion_dst, dilation_dst;
Mat imgIn, imgOut;
Mat imgOn, imgOff ,imgDiff;
#define PI 3.14159265359

int lower_hue_range;
int upper_hue_range;


int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

/// ///////////////////////////////////////////////////////////////////
///																	//
///	GESTIONE  WEBCAM												//
///																	//
/// ///////////////////////////////////////////////////////////////////

	///ABILITARE UNA DELLE 3 OPZIONI
	//#define IPCAM
	//	const std::string videoStreamAddress = "http://192.168.0.20/img/video.mjpeg";
	#define USBCAM
	//#define RASPICAM

	#define IMG_W 320
	#define IMG_H 240
	int cameraFPS = 8; //Rate di acquisizione della webcam, pari al doppio del laser
	int camera_fov_vert_deg =  32;
	int camera_fov_horiz_deg =  90; //verificato con /scan ; con qs angolo restituisce una retta
	float rpc =0.0; /// radians_per_pixel pitch : calcolato dinamicamente in base ai parametri di calbrazione

	/// DEFINIZIONE  GLOBALE DELL' OGGETTO WEBCAM
	#ifdef RASPICAM	
		/// Raspicam-------------------------------------
		raspicam::RaspiCam_Cv vcap;
	#endif

	#ifdef USBCAM
		VideoCapture vcap;		 
	#endif

	#ifdef IPCAM
		VideoCapture vcap;		 
	#endif
	/// imposta il frame rate di acquisizione
	void cameraSetFPS(int fps)
	{
		vcap.set(CV_CAP_PROP_FPS, fps);
	}
	void OnChangeCameraFrequency(int, void*){
		cameraSetFPS(cameraFPS);
		printf( "Set Camera FPS %i Hz\n" ,cameraFPS );
	}
	/// ritorna true se APRE il device di cattura: vcap.open()
	bool setupWebcam(){
		#ifdef RASPICAM	
			/// Raspicam-------------------------------------
			//raspicam::RaspiCam_Cv raspicam; //spostata nelle definizioni globali
			vcap.set ( CV_CAP_PROP_FRAME_WIDTH,  IMG_W );
			vcap.set ( CV_CAP_PROP_FRAME_HEIGHT, IMG_H );
			cameraSetFPS(cameraFPS);
			
			//    vcap.set ( CV_CAP_PROP_BRIGHTNESS,getParamVal ( "-br",argc,argv,50 ) );
			//    vcap.set ( CV_CAP_PROP_CONTRAST ,getParamVal ( "-co",argc,argv,50 ) );
			//    vcap.set ( CV_CAP_PROP_SATURATION, getParamVal ( "-sa",argc,argv,50 ) );
			//    vcap.set ( CV_CAP_PROP_GAIN, getParamVal ( "-g",argc,argv ,50 ) );
			
			if ( !vcap.open() ) {
				return false;
			}	 
		#endif

		#ifdef USBCAM
			//acquisizione webcam USB
			vcap.open(0);
			//vcap.read(imgOut); 
			if (!vcap.isOpened()) {
				return false;
			}
			vcap.set ( CV_CAP_PROP_FRAME_WIDTH,  IMG_W );
			vcap.set ( CV_CAP_PROP_FRAME_HEIGHT, IMG_H );
			cameraSetFPS(8);
		 #endif

		#ifdef IPCAM
			//open the video stream and make sure it's opened
			if (!vcap.open(videoStreamAddress)) { // IpCamera Linksys
				return false;
			} 	
			vcap.set ( CV_CAP_PROP_FRAME_WIDTH,  IMG_W );
			vcap.set ( CV_CAP_PROP_FRAME_HEIGHT, IMG_H );
			cameraSetFPS(8);
		#endif	
		return true;
		
	}

	// restituisce l'acquisizione dell'immagine in base alla webcam attiva
	cv::Mat getImg(){
		cv::Mat img;
		#ifdef RASPICAM
			vcap.grab();
			vcap.retrieve ( img );
		#endif
		
		#ifdef USBCAM
		
			vcap.grab(); //get next videoframe
			vcap.retrieve ( img );
				
			// vcap.read(img);
		#endif
		
		#ifdef IPCAM
		
			vcap.grab(); //get next videoframe
			vcap.retrieve ( img );
		#endif
		return img;
	}

	//std::mutex mtxCam;
	void threadCapture(VideoCapture *cap, Mat *frame) /* esempio di attivazione :    thread t(threadCapture, &cap, &frame); */
		{
			//std::mutex mtxCam;
			while (true) 
			{
				//mtxCam.lock();
				*cap >> *frame;
				//mtxCam.unlock();
				imshow("Image thread",*frame);
				waitKey(1);
			}
		}
		
	/*
	int detectNumAttachedCvCameras()
	{
		int number = 0;
		while (true)
		{

			vcap.open(number);
			if (!vcap.isOpened())
			{
				std::cout << "Detected cameras: "<< number << std::endl;
				printf("ok");
				break;
			}
			else
			{
				number++;
				vcap.release();
			}
		}
		return number;
	}*/

	void cameraSetExposure(int, void*)
	{

		vcap.set(CV_CAP_PROP_EXPOSURE, cameraExposure-7);
		/*cmeraParameter
		15 = Esposizione
		CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
		CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
		CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
		CV_CAP_PROP_HUE Hue of the image (only for cameras).
		14= CV_CAP_PROP_GAIN Gain of the image (only for cameras).
		15= CV_CAP_PROP_EXPOSURE Exposure (only for cameras).
		CV_CAP_PROP_CONVERT_RGB
		*/
	}
	void cameraSetGain(int, void*)
	{
		vcap.set(CV_CAP_PROP_GAIN, cameraGain-64);
	}
	void cameraSetResolution( )
	{
		vcap.set(CV_CAP_PROP_FRAME_WIDTH,IMG_W);
		vcap.set(CV_CAP_PROP_FRAME_HEIGHT,IMG_H);
	 }

/// ///////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////


/// ///////////////////////////////////////////////////////////////////
///																	//
///	ROS SCAN 														//
///																	//
/// ///////////////////////////////////////////////////////////////////
#pragma region ROS_SCAN
	#include <sensor_msgs/LaserScan.h>


	//--------------------------------
	sensor_msgs::LaserScan scan_msg;;
	
	ros::Publisher pub_Laser;//ros::Publisher pub_Laser("scan", &scan_msg);
	
	#define LDSsamples IMG_W
	#define LDSspeed PI		// PI rad/sec = 180°/sec 
	#define SCAN_ANGLE PI/2		//SCAN_ANGLE_MAX - SCAN_ANGLE_MIN
	#define SCAN_ANGLE_MIN  -SCAN_ANGLE/2	//The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
	#define SCAN_ANGLE_MAX   SCAN_ANGLE/2

	#define SCAN_ANGLE_INCREMENT  SCAN_ANGLE / LDSsamples;  // (SCAN_ANGLE_MAX - SCAN_ANGLE_MIN)/ LDSsamples 
	#define SCAN_TIME  2 // LDSmicrostepsPerStep* 2* (SCAN_ANGLE_MAX-SCAN_ANGLE_MIN )/LDSspeed
	#define SCAN_TIME_INCREMENT_MS  SCAN_TIME/LDSsamples
	#define	SCAN_TIME_INCREMENT SCAN_TIME_INCREMENT_MS/1000f		// (1 / 100) / (LDSsamples) //0.0667
	#define	SCAN_RANGE_MIN 0.01f
	#define	SCAN_RANGE_MAX 8.0f

	//	float ranges[LDSsamples]; // max of 30 measurements
	float intensities[LDSsamples]; // buffer 
								   //------------------------
	//	unsigned long scan_time;


	void setup_ROS_LaserScan() {
		

		// create the test data
		scan_msg.header.frame_id = "scan"; //The laser is assumed to spin around the positive Z axis (counterclockwise, if Z is up) with the zero angle forward along the x axis
		scan_msg.angle_min = -SCAN_ANGLE / 2;
		scan_msg.angle_max = SCAN_ANGLE / 2;
		scan_msg.angle_increment = (float)SCAN_ANGLE / LDSsamples;	// o horiz_fov_rad /IMG_W  ?
		scan_msg.scan_time = 0;	// 3.91 tempo reale di scansione con velocità impostata di PI ; dovrebbe essere così ma ci mette quasi 4 sec. a mezzo giro  (float) SCAN_ANGLE / LDSspeed ;
		scan_msg.time_increment = scan_msg.scan_time / (float)(LDSsamples - 1);	//(1 / laser_frequency) / (num_readings);
		scan_msg.range_min = SCAN_RANGE_MIN;
		scan_msg.range_max = SCAN_RANGE_MAX;
		scan_msg.ranges.resize(LDSsamples);
		scan_msg.intensities.resize(LDSsamples);
		for (int z = 0; z < LDSsamples; z++)
		{
			scan_msg.ranges[z] = 1.0;
			scan_msg.intensities[z] = 1.0;
		}
		
		

	}

	// pubblica
	void publish_laserscan( ) {

		scan_msg.header.stamp = ros::Time::now();
		
		pub_Laser.publish(scan_msg);// pub_Laser.publish(&scan_msg);
	}

#pragma endregion


/// ///////////////////////////////////////////////////////////////////
///																	//
///	ALGORITMI LDS													//
///																	//
/// ///////////////////////////////////////////////////////////////////


	//////////////////////////////////
	//								//
	//	POSIZIONAMENTO  WEBCAM		//
	//								//
	//////////////////////////////////
	/// webcam angular offset (compensates for alignment errors)
	int rOffsetDeg = 20; /// Inclinazione in gradi della webcam dall'asse orizz.
	float rOffset = rOffsetDeg/180*3.141592; /// Radian offset
	/// distance between laser and camera in mm
	int LaserCameraDistmm = 110;



	//////////////////////////////////
	//								//
	//	CALIBRAZIONE		//
	//								//
	//////////////////////////////////
	int calibrationStartRow = 10;
	int calibrationStartRowDistanceCm = 600;
	int calibrationLastRowDistanceCm = 20;
	cv::Point calibPt1 = Point(0, calibrationStartRow );
	cv::Point calibPt2  = Point(IMG_W, calibrationStartRow);

	//////////////////////////////////
	//								//
	//	OUTPUT						//
	//								//
	//////////////////////////////////
	//float dist[IMG_W]; //vettore delle distanze
	float *ranges = new float[IMG_W];

	int threshval =60;

	#define DEG2RAD 0.01745329252


	///converte la colonna in angolo, sulla base dei parametri IMG_W , horiz_fov_rad della webcam
	float imgCol2Alfa(int col){

		if (col < 0 )	{col =0;}
		if (col > IMG_W ){col = IMG_W;}

		const float_t horiz_fov_rad = (float_t)camera_fov_horiz_deg * DEG2RAD; // 3.14159265359/180;// camera Horizontal field of view

		float pixel2rad = horiz_fov_rad /IMG_W;// rpc =  radians_per_pixel pitch

		int pfc = IMG_W/2 -col ; // pixel from center 

		float alfa = pfc *  pixel2rad ;
		return alfa;

	}
	/*
	 * ritorna la riga centrale al laser
	 * in: Immagine
	 * indice riga
	 */

	int findLaserRow(Mat img, int column){
		int num=0; ///numeratore
		int den=0; //denomin.
		int rowFound =0; // riga da restituire

		int pxl =0 ;  //pixel value
		/// calcola la riga come media

		int mn = 255;
		int mx = 0;
		bool  lookformax =true;
	   int mnpos = -1;
	   int mxpos = -1;

		std::vector<int> maxtab;
		std::vector<int> mintab;

		std::vector<int> myVect;
		for( int i = 0; i < IMG_H ; i++ )
			myVect.push_back( i );


		int delta = threshval; // altezza del picco



		/// Per ogni riga
		for(int r=0; r< img.rows ; r++){	// in vector:

			//legge il pixel	-----------------------------------

			if (img.type() == CV_8UC1) { // gray-level image

				pxl = (int)img.at<uchar>(r,column);
			}
			else if (img.type() == CV_8UC3) { // color image
				// OPENCV usa codifica BGR
				pxl = (int)img.at<cv::Vec3b>(r,column)[1];	//canale verde

			}
			//-------------------------------------------------------



			// ricerca del primo picco ------------
			if (pxl > mx){
				mx = pxl;
				mxpos = r;
			}
			if (pxl < mn){
				mn = pxl;
				mnpos = r;
			}


			if (lookformax){
				if (pxl < mx-delta){
					lookformax = false; //trovato il picco
					rowFound = r;
					mn = pxl; //?


				}
			}


			//------------------------


			//debug
			//cout <<"\nr:"<<r<<"  "<< pxl;


			//calcolo del baricentro facendo una media pesata delle righe ,
			// dando come peso l'intensità del pixel in corrispondenza della riga
			num += (r * pxl);  //accumulo il numeratore
			den += pxl ;//  # somma dei pesi= valori dei pixel sopra soglia

		} //end loop

	/*
		if(den >0){ //ci sono pixel sopra soglia nella colonna corrente?
			rowFound = round(num / den);	//  #assegno all'elemento del vettore la posizione schermo
		}else{
			rowFound = -1;
		}
	*/
		// limitatore 
		if (	rowFound <0)
		{
			rowFound =0;
		}
		if (	rowFound > IMG_H)
		{
			rowFound =IMG_H;
		}
		
		return rowFound;

	}
	/*
	 * Converte l'indice di riga in distanza
	 * in : immagine [mat] e colonna
	 * out: distanza in cm in corrispondenza della colonna del picco luminoso
	 * */
	float imgRow2Dist_orig(int row){
		// r=0 massima distanza
		// r = IMG_H  minima distanza

		// come usare calibrationStartRowDistanceCm ?
		//				e calibrationLastRowDistanceCm ?
		float d =0.0;



		if(row>=0){

			/// pixel from center (pfc) of focal plane
			//orig pfc = abs(laser_row_i - height);
			int pfc =row - IMG_H/2 ;
			//cout << "pfc= " << pfc <<"\n";

			/// angolo rispetto l'orizzonte = angolo Webcam + angolo linea laser dal centro
			float alfa =rOffset + (float)pfc* rpc ;
			//cout << "alfa= " << alfa <<"\n";

			/// divido per 1000 perchè LaserCameraDist è in mm e la distanza va data in metri
			d = LaserCameraDistmm/tan(alfa)/1000;





		}else{
			d=SCAN_RANGE_MAX;///  no laser could be detected in this column
		}
		return d;

	}


	/// calcola la distanza tenendo conto anche dell'angolo (passato tramite il numero di colonna)
	float imgRow2Dist(int row, int col){
		// r=0 massima distanza
		// r = IMG_H  minima distanza

		// come usare calibrationStartRowDistanceCm ?
		//				e calibrationLastRowDistanceCm ?
		float d =0.0;



		if(row>=0){

			/// pixel from center (pfc) of focal plane
			//orig pfc = abs(laser_row_i - height);
			int pfc =row - IMG_H/2 ;
			//cout << "pfc= " << pfc <<"\n";

			/// angolo rispetto l'orizzonte = angolo Webcam + angolo linea laser dal centro
			float alfa =rOffset + (float)pfc* rpc ;
			//cout << "alfa= " << alfa <<"\n";

			/// divido per 1000 perchè LaserCameraDist è in mm e la distanza va data in metri
			d = LaserCameraDistmm/tan(alfa)/1000;

			/// distanza corretta in base all'angolo orizzontale 
			d = d/cos(imgCol2Alfa(col));



		}else{
			d=SCAN_RANGE_MAX;// -1 =  no laser could be detected in this column
		}
		return d;

	}

	 // ritorna la distanza misurata in corrispondenza della colonna indicata
	float ldsGetDist(Mat img, int col, Mat imgOut){
		
		/// inizializza la distanza a quella massima
		float d = SCAN_RANGE_MAX;
		int row = findLaserRow(img, col); /// deve restituire un indice di riga > calibrationStartRow
		
		if ( row >= calibrationStartRow ) ///situazione di calcolo ok
		{
			d= imgRow2Dist(row,col); /// converte la riga in distanza				
		}
		
		/// display del punto riconosciuto
		imgOut.at<cv::Vec3b>(row,col)[2] =255;
		return d;	
		
		



		// evidenzia il pixel
		//imgOut.row(col).setTo(Scalar(0));

		// BGR	0=blu |2= red

		// x debug
		/*
		 //	cout << "\nrow: " << row << ", col" << column ;

		if (img.type() == CV_8UC1) { // gray-level image

			img.at<uchar>(row,column) =255;
		}
		else if (img.type() == CV_8UC3) { // color image
			// BGR
			img.at<cv::Vec3b>(row,column)[0] =0;
			img.at<cv::Vec3b>(row,column)[1] =255;
			img.at<cv::Vec3b>(row,column)[2] =0;
		}

		*/



	 }




	/// //////////////////////////////////////////////////////////////
	/// per ogni colonna della matrice immagine calcola la distanza
	/// in: img immagine differenza
	/// out vettore delle distanze im metri dist[IMG_W]
	/// //////////////////////////////////////////////////////////////
	void ldsProcessImg(Mat imgIn, Mat imgOut)
	{

		/// per ogni colonna calcola la distanza
		/// la scansione deve essere in senso antiorario
	   for(int c=imgIn.cols-1; c>=0; c--){

			scan_msg.ranges[imgIn.cols-1-c]= ldsGetDist( imgIn, c, imgOut );
			
			/// output for debug
			//~ if ((debugOn) && (scan_msg.ranges[c] < SCAN_RANGE_MAX))
			//~ {
				//~ cout <<"\ncol:"  <<c <<" d:"<< scan_msg.ranges[c];
				//~ 
			//~ }
			//~ 
		}

	}




	void UpdateCalibrationParameters(){
		/// calcolo di rpc e rOffset in base ai parametri di calibrazione
		 /// rpc =  radians per pixel pitch (180 deg=pi rad)

		//rpc =CV_PI * ((float)camera_fov_vert_deg / 180)/(float)IMG_H;

		rpc = ( atan((float)LaserCameraDistmm/1000) /((float)calibrationStartRowDistanceCm /100)- atan((float)LaserCameraDistmm/1000) /((float)calibrationLastRowDistanceCm /100)   )
				 / (((float)calibrationStartRow - IMG_H));
		rOffset  = 0.5 * (
						atan((float)LaserCameraDistmm/1000) /((float)calibrationStartRowDistanceCm /100)
						+ atan((float)LaserCameraDistmm/1000) /((float)calibrationLastRowDistanceCm /100)
						-  ( rpc* (float)calibrationStartRow )
							);
		rOffsetDeg = (int)(rOffset *180/CV_PI);

		//----------------------------
		}
	void OnCalibrationParameterChange(int, void*){

		 /// rpc =  radians per pixel pitch (180 deg=pi rad)
		rpc =CV_PI * ((float)camera_fov_vert_deg / 180)/(float)IMG_H;
		cout << "rpc= " << rpc <<"\n";

		rOffset = (float)rOffsetDeg/180*3.141592;
		cout << "rOffset= " << rOffset <<"\n";

		 UpdateCalibrationParameters();
		 cout << "rpc: " << rpc << ", rOffset:" << rOffset << " ( " << rOffsetDeg << "°)\n";

	}
	void OnDebugOnOff(int, void*){
		cout << "\nDebug: " << debugOn;
		
		}
	void OnrOffsetDegChange(int, void*){



		rOffset = (float)rOffsetDeg/180*3.141592;
		cout << "rOffset= " << rOffset <<"\n";

		 UpdateCalibrationParameters();
		 cout << "rpc: " << rpc << ", rOffset:" << rOffset << " ( " << rOffsetDeg << "°)\n";

	}
	 //--------------------------------------------------------------------------------


/// ////////////////////////////////////////////////////////////////////
///								
///	CV_BRIDGE					
///								
/// ////////////////////////////////////////////////////////////////////
	#include <sensor_msgs/Image.h>
	#include <sensor_msgs/image_encodings.h>
	#include <image_transport/image_transport.h>
	#include <cv_bridge/cv_bridge.h>

//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


/// ////////////////////////////////////////////////////////////////////
///								
///	MAIN				
///								
/// ////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
	//////////////////////////////////
	//								//
	//	SETUP ROS					//
	//								//
	//////////////////////////////////

	ros::init(argc, argv, "rm_scan_node");
	ros::NodeHandle nh;
	
	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img_msg; // >> message to be sent
	//~ ros::Publisher pub_img ; // sostituito da  image_transport::Publisher 

	//~ sensor_msgs::CompressedImage imgCompressed_msg;
	//~ ros::Publisher pub_imgCompressed ;

	/// Per pubblicare immagini consigliano di usare imageTansport
	/// http://wiki.ros.org/image_transport
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub_img = it.advertise("/webcam", 1);

	//~ image_transport::Publisher pub_imgCompressed = it.advertise("/webcam", 1);
	//~ pub_img   = nh.advertise<sensor_msgs::Image>("/webcam", 2);	
	//~ pub_imgCompressed  = nh.advertise<sensor_msgs::CompressedImage>("/webcamCompressed", 2);	


	pub_Laser = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);//nh.advertise(pub_Laser);


	
	ROS_INFO("setup_ROS_LaserScan");
	setup_ROS_LaserScan();
	
	ros::Rate loop_rate(4); // valore = frequenza in Hz
	
	/// ///////////////////////////////
	///	SETUP LASER					
	/// ///////////////////////////////
	ROS_INFO("Testing Laser on/off");

	wiringPiSetup () ;
	initLaser();
	testLaser();


	//////////////////////////////////
	//								//
	//	SETUP openCV				//
	//								//
	//////////////////////////////////
	ROS_INFO("SETUP openCV");
	Mat img;
	//imgOut = Mat.zeros(IMG_H,IMG_W, CvType.CV_8UC3 , Scalar.all(0));
	imgOut = cv::Scalar(0, 0, 0); //bgr
	//	cameraSetResolution();
	cv::namedWindow("wImageOn", 1);
	cv::namedWindow("wImageOut", 1);
	cv::namedWindow("wControls", 1);	
	//	CV::namedWindow(window_name, CV_WINDOW_AUTOSIZE);
	
	

	
	//////////////////////////////////
	//								//
	//	controlli M M I 			//
	//								//
	//////////////////////////////////
	//	setMouseCallback("wImageOut", onMouse, reinterpret_cast<void*>(&imgDiff));
	createTrackbar("Laser freq Hz  ", "wControls", &LaserFreq, 33, OnChangeLaserFrequency );
	createTrackbar("Camerafreq Hz  ", "wControls", &cameraFPS, 33, OnChangeCameraFrequency );
	createTrackbar("Debug Off-On", "wControls", &debugOn, 1, OnDebugOnOff);
	createTrackbar("camera_fov_horiz_deg", "wControls", &camera_fov_horiz_deg, 120);
	createTrackbar("camera_fov_vert_deg", "wControls", &camera_fov_vert_deg, 90);
	createTrackbar("rOffsetDeg", "wControls", &rOffsetDeg, 90);

	//~ createTrackbar("HUE max", "wControls", &upper_hue_range , 360);
	//~ createTrackbar("HUE min", "wControls", &lower_hue_range , 360);
 	//~ createTrackbar("After ON delay", "wControls", &LASER_ON_DELAY , 100);
	//~ createTrackbar("After OFF delay", "wControls", &LASER_OFF_DELAY , 100);
	createTrackbar("Threshold", "wControls", &threshval, 255);


	// controlli per la calibrazione
	createTrackbar("start row ", "wControls", &calibrationStartRow , 100, OnCalibrationParameterChange);
 	createTrackbar("start row dist [cm]", "wControls", &calibrationStartRowDistanceCm , 700, OnCalibrationParameterChange);
 	createTrackbar("last row dist [cm]", "wControls", &calibrationLastRowDistanceCm , 50, OnCalibrationParameterChange);

 	createTrackbar("fcamera_fov_vert_deg  [deg]", "wControls", &camera_fov_vert_deg  , 130, OnCalibrationParameterChange);
 	createTrackbar("rOffsetDeg [deg]", "wControls", &rOffsetDeg  , 90, OnrOffsetDegChange);

	
	
	// inizializzazione variabili

	//	int threshval = 20;
	upper_hue_range = 90;
	lower_hue_range = 70;
	//----------------------------


	// inizializzazione parametri di calibrazione
	UpdateCalibrationParameters();


	
	//////////////////////////////////
	//								//
	//	APERTURA WEBCAM				//
	//								//
	//////////////////////////////////

	ROS_INFO("Connecting to camera");
	if ( setupWebcam() ) ROS_INFO("Connected to camera"); 
	else {
		ROS_ERROR("FATAL Error opening webcam");
		vcap.release();
		return 0;
	}


	


	/**
	 * This is a message object. You stuff it with data, and then publish it.
	 */
	std_msgs::String msg;

	/// disatttivo il laser
	//~ laserOff(LASER_OFF_DELAY);


	//////////////////////////////////////////////////////////////////////
	//																	//
	//	MAIN LOOP														//
	//																	//
	//////////////////////////////////////////////////////////////////////
	int loopCount = 0;
	float alfa = -1; // angolo in radianti del volto ; -1 se non ci sono volti
	while (ros::ok())
	{
		if (debugOn){
			std::stringstream ss;
			ss << "loop " << loopCount;
			msg.data = ss.str();
			ROS_INFO("%s \n", msg.data.c_str());
		}


	
		///acquisizione di due frame consecutivi 
		imgOff = getImg(); //was; vcap.read(imgOff);
		imgOn = getImg(); //was; vcap.read(imgOn);

		///differenza
		//~ imgDiff = imgOn - imgOff;
		cv::absdiff(imgOn, imgOff, imgDiff);
		

		
		/// FILTRAGGIO IMMAGINE
		
		if (	threshval > 0 )
		{
			//threshold(imgDiff,imgDiff, threshval,200 , cv::THRESH_BINARY )	;
			threshold(imgDiff,imgOut, threshval,255 , 3)	; // 3 threshold to 0 se < threshval
			
			/*
			//conversione bianco e nero
			//		cvtColor( imgDiff, imgDiff, CV_BGR2GRAY );

			// filtro lasciando passare solo il canale verde
			cv::inRange(imgOn, cv::Scalar(20, 254, 254), cv::Scalar(255, 255, 255), imgDiff);
			cv::inRange(imgDiff, cv::Scalar(20, 254, 254), cv::Scalar(255, 255, 255), imgDiff);
			threshold(imgDiff,imgDiff, threshval,200 , cv::THRESH_BINARY )	;
			cv::Canny(imgDiff, imgDiff,125,350);
			*/
							
		}
	


		
		///elaborazione
		ldsProcessImg(imgDiff,imgOn);


		/// traccio le linee di calibrazione
		calibPt1.y = calibrationStartRow;
		calibPt2.y = calibrationStartRow;
		line(imgOn,  calibPt1,  calibPt2, (0,0,250) );
		
		
		///visualizzazione immagini
		
			imshow("wImageOn", imgOn);
		if (	debugOn==1)
		{
			imshow("wImageOut", imgOut);
			
		}
		
		/// debug ------------------------
		//ss << "\n alfa " << alfa;
		//msg.data = ss.str();
		//ROS_INFO("%s", msg.data.c_str());




		//////////////////////////////////
		//								//
		//	ROS PUBLISH SCAN			//
		//								//
		//////////////////////////////////

 		publish_laserscan();
 		
 		
 		
 		
 		
		//////////////////////////////////
		//								//
		//	ROS_bridge PUBLISH IMAGE	//
		//								//
		//////////////////////////////////
/**/
  		/// https://stackoverflow.com/questions/27080085/how-to-convert-a-cvmat-into-a-sensor-msgs-in-ros
		std_msgs::Header header; /// empty header
		header.seq = loopCount; /// user defined counter
		header.stamp = ros::Time::now(); // time
		
		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, imgOn); //con sensor_msgs::image_encodings::RGB8 ottengo colori fasulli in rviz
		
		//~ img_bridge.toCompressedImageMsg(imgCompressed_msg);
 		//~ pub_imgCompressed.publish(imgCompressed_msg);
 		
		img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
		pub_img.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);

 		//////////////////////////////////
		//								//
		//	QUIT ON USER REQUEST		//
		//								//
		//////////////////////////////////
		char key = (char)waitKey(30); //delay N millis, usually long enough to display and capture input
		switch (key) {
		case 'q':
		case 'Q':
		case 27: //escape key
				laserOff();
				setLaserFreq(0);
				vcap.release();

		
			return 0;
		default:
			break;
		}

 		//////////////////////////////////
		//								//
		//	ROS END LOOP PROCESSING		//
		//								//
		//////////////////////////////////
		
		ros::spinOnce();

		loop_rate.sleep();		
		++loopCount;

	} // end while Ros::ok
	
	#ifdef RASPICAM	
		vcap.release();
	#endif
	return 0;
}
