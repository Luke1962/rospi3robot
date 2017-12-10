// TEST FACE DETECTION 

#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdio.h>
#include <raspicam/raspicam_cv.h>


#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include <sstream>

// dimensioni immagine acquisita
#define IMG_W 320
#define IMG_H 240

using namespace std;
using namespace cv;


/** Global variables */
String face_cascade_name = "/home/pi/ros_catkin_ws/src/myrobotmaster/src/rm_head_tracking/src/haarcascade_frontalface_alt.xml";
String eyes_cascade_name = "/home/pi/ros_catkin_ws/src/myrobotmaster/src/rm_head_tracking/src/haarcascade_frontalface_alt2.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
string window_name = "rm_head_tracking_node";

RNG rng(12345);
int i=0;

//converte la colonna in angolo, sulla base dei parametri della webcam
float imgCol2Alfa(int col){
	
	if (	col < 0 )
	{
		col =0;
	}
	
	if (	col > IMG_W )
	{
		col = IMG_W;
	}
	
	const float_t horiz_fov_rad = 3.14159265359/2;// camera Horizontal field of view  
    
    float rpc = horiz_fov_rad /IMG_W;// rpc =  radians per pixel pitch 
 
	int pfc = IMG_W/2 -col ; // pixel from center
	
	float alfa = pfc *  rpc ;	
	return alfa;

}

// SE riconosce un volto ,ritorna l'angolo in radianti rispetto al centro (<0 se a DX)
// altrimenti ritorna -10
float detectHeadHorizAngle( Mat frame )
{
	std::vector<Rect> faces;
	Mat frame_gray;

	cvtColor( frame, frame_gray, CV_BGR2GRAY );
	equalizeHist( frame_gray, frame_gray );

	int FaceMaxSizeX = 0; // dimensione del volto più grande
	int FaceMaxSizeIndex = -1; // indice del volto piu grande
	
	//-- Detect faces
	face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );


	// poichè possono esserci più volti sceglie quello più grande, ovvero il più vicino
	for( size_t i = 0; i < faces.size(); i++ )
	{
		
		//		Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );		 
		//		ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

		if (	faces[i].width > FaceMaxSizeX )
		{
			FaceMaxSizeIndex = i;
		}
		
	}
	
	// se ha trovato ritorna l'angolo
	if (FaceMaxSizeIndex >-1 ) 
	{
		return imgCol2Alfa(faces[FaceMaxSizeIndex].x);
	}else 
	{
		return -10;	//non trovato
	}
	
	//-- Show what you got
	//imshow( window_name, frame );
	

}

/** @function detectAndDisplay originale */
void ___detectAndDisplay( Mat frame )
{
  std::vector<Rect> faces;
  Mat frame_gray;

  cvtColor( frame, frame_gray, CV_BGR2GRAY );
  equalizeHist( frame_gray, frame_gray );

  //-- Detect faces
  face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

  for( size_t i = 0; i < faces.size(); i++ )
  {
    Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
     
    ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

  }
  //-- Show what you got
  imshow( window_name, frame );

}

 
 

 /** @function main */
 int _main( int argc, const char** argv )
 {
	 
	namedWindow(window_name, CV_WINDOW_AUTOSIZE);
	Mat img;
		 
	cout<<"Connecting to camera"<<endl;
	raspicam::RaspiCam_Cv Camera;
    Camera.set ( CV_CAP_PROP_FRAME_WIDTH,  IMG_W );
    Camera.set ( CV_CAP_PROP_FRAME_HEIGHT, IMG_H );
//    Camera.set ( CV_CAP_PROP_BRIGHTNESS,getParamVal ( "-br",argc,argv,50 ) );
//    Camera.set ( CV_CAP_PROP_CONTRAST ,getParamVal ( "-co",argc,argv,50 ) );
//    Camera.set ( CV_CAP_PROP_SATURATION, getParamVal ( "-sa",argc,argv,50 ) );
//    Camera.set ( CV_CAP_PROP_GAIN, getParamVal ( "-g",argc,argv ,50 ) );
	
    if ( !Camera.open() ) {
        cerr<<"Error opening camera"<<endl;
        return -1;
    }
    cout<<"Connected to camera ="<<Camera.getId() <<endl;


   


   //-- 1. Load the cascades
   if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };
   if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading\n"); return -1; };

   //-- 2. Read the video stream



     while( true )
     {
 
		// versione per raspicam------
 		Camera.grab();
        Camera.retrieve ( img );

        //----------------------------   
        	
		//-- 3. Apply the classifier to the frame
		___detectAndDisplay( img ); 

 
 
 
 
        if (waitKey(20) >= 0){
			Camera.release();
            break;
        }
      }

 }


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

	
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* For programmatic remappings you can use a different version of init() which takes
	* remappings directly, but for most command-line programs, passing argc and argv is
	* the easiest way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "rm_head_tracking_node");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	/**
	* The advertise() function is how you tell ROS that you want to
	* publish on a given topic name. This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing. After this advertise() call is made, the master
	* node will notify anyone who is trying to subscribe to this topic name,
	* and they will in turn negotiate a peer-to-peer connection with this
	* node.  advertise() returns a Publisher object which allows you to
	* publish messages on that topic through a call to publish().  Once
	* all copies of the returned Publisher object are destroyed, the topic
	* will be automatically unadvertised.
	*
	* The second parameter to advertise() is the size of the message queue
	* used for publishing messages.  If messages are published more quickly
	* than we can send them, the number here specifies how many messages to
	* buffer up before throwing some away.
	*/
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(0.5); // valore = frequenza in Hz
	
	
	
	
	
	/// OPENCV SETUP ---
	namedWindow(window_name, CV_WINDOW_AUTOSIZE);
	Mat img;


	/// CAMERA SETUP ------------------------------------------------------------
		 
	ROS_INFO("Connecting to camera");
	
	raspicam::RaspiCam_Cv Camera;
    Camera.set ( CV_CAP_PROP_FRAME_WIDTH,  IMG_W );
    Camera.set ( CV_CAP_PROP_FRAME_HEIGHT, IMG_H );
	//    Camera.set ( CV_CAP_PROP_BRIGHTNESS,getParamVal ( "-br",argc,argv,50 ) );
	//    Camera.set ( CV_CAP_PROP_CONTRAST ,getParamVal ( "-co",argc,argv,50 ) );
	//    Camera.set ( CV_CAP_PROP_SATURATION, getParamVal ( "-sa",argc,argv,50 ) );
	//    Camera.set ( CV_CAP_PROP_GAIN, getParamVal ( "-g",argc,argv ,50 ) );

    if ( !Camera.open() ) {
        ROS_FATAL("Error opening camera");
        return -1;
    }
    ROS_INFO("Connected to camera");

	///----------------------------------------------------------------------
	
	///test 
	Camera.grab();
	Camera.retrieve ( img );
	
	// sembra necessario eseguire imgshow entro un certo tempo
	// dopo nameWindow() , altrimenti da errore runtime
	// GLib-GObject-CRITICAL **: g_object_unref: assertion 'G_IS_OBJECT (object)' failed
	imshow( window_name, img );


	///-- 1. Load the cascades
	if( !face_cascade.load( face_cascade_name ) ){ ROS_FATAL("--(!)Error loading\n"); return -1; };
	if( !eyes_cascade.load( eyes_cascade_name ) ){ ROS_FATAL("--(!)Error loading\n"); return -1; };
	
	
	
	
	
	
	

	/**
	 * This is a message object. You stuff it with data, and then publish it.
	 */
	std_msgs::String msg;



	int count = 0;
	float alfa = -1; // angolo in radianti del volto ; -1 se non ci sono volti
	while (ros::ok())
	{
		std::stringstream ss;	  
 
		ss << "loop " << count;
		msg.data = ss.str();
		ROS_INFO("%s \n", msg.data.c_str());

		/// Elaborazione --------------------------------  
		  
		// versione per raspicam------
		Camera.grab();
		Camera.retrieve ( img );
		//----------------------------   

		///-- Show what you got
		imshow( window_name, img );
		ROS_INFO("ELABORO");	
		///-- 3. Apply the classifier to the frame
		alfa = detectHeadHorizAngle( img ); 

		/// debug ------------------------	
		ss << "\n alfa " << alfa;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());
		
		
		///-----------------------------------------------
		#define SOGLIA_CENTRO 0.05
		if (alfa == -10	) //nessun volto
		{
			// non c'è nessuno ?
			//ss << "non cè nessuno ?";
			msg.data = "";
		}else if (abs(alfa) <= SOGLIA_CENTRO 	)
		{
			//centro
			msg.data ="centro";

		}else if (	alfa < SOGLIA_CENTRO )
		{
			//DESTRA
			msg.data = "destraa";
			
			
			
		}else if (alfa > SOGLIA_CENTRO	)
		{
			//SINISTRA
			msg.data = "siniistra";
			
		}
		
		
		
		//msg.data = ss.str();
		/**
		 * The publish() function is how you send messages. The parameter
		 * is the message object. The type of this object must agree with the type
		 * given as a template parameter to the advertise<>() call, as was done
		 * in the constructor above.
		 */
		chatter_pub.publish(msg);

		

		// premere un tasto per terminare
		if (waitKey(20) >= 0){
			Camera.release();
			break;
		}
		
	
	  



		ros::spinOnce();

		loop_rate.sleep();
		++count;
		
	} // end while Ros::ok

	Camera.release();
	return 0;
}
