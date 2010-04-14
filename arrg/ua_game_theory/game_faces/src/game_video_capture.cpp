// University of Arizona
// game_faces
// Reads the messages from the game
// and creates the video and logfile

// THIS PROGRAM IS RUN JUST ONCE DURING THE WHOSE SESSION
// IT IS NOT CLOSED UNTIL THE EXPERIMENT IS DONE

#define CV_NO_BACKWARD_COMPATIBILITY

// OpenCV stuff
#include <opencv/cv.h>
#include <opencv/highgui.h>

// ROS stuff
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "game_faces/TwoPersonGame.h"
#include "game_faces/GamePlay.h"

// C++ stuff
#include <sstream>
#include <iostream>
#include <cstdio>
#include <string>

#ifdef _EiC
#define WIN32
#endif

using namespace std;
using namespace cv;

int frame_id=0;
int player_id = 0;
String Logmsg; 
ros::Subscriber game_sub;


int videoCaptureThread();

template <class T>
inline std::string to_string (const T& t)
{
	std::stringstream ss;
	ss << t;
	return ss.str();
}

void gamePlayCallback(const game_faces::GamePlayConstPtr& msg)
{
    ROS_INFO("Received game play number: %d, amount: %d", msg->play_number, msg->amount);
}

void gameStartCallback(const game_faces::TwoPersonGameConstPtr& msg)
{
    ROS_INFO("Received game topic: [%s]", msg->game_topic.c_str());
    ros::NodeHandle n;
	game_sub = n.subscribe(msg->game_topic, 100, gamePlayCallback);
}



int main( int argc,  char** argv )
{

    if(argc < 2){
        cerr << "Error: Must pass in the player_id" << endl;
    }
    // Not very robust but just assume the first argument is the player_id
    int player_id = atoi(argv[1]);
    
	// Will publish when the video is recording
	ros::init(argc, argv, String("Video") + to_string(player_id));
	//ros::NodeHandle n;
	//ros::Publisher video = n.advertise<std_msgs::String>("Video_signal", 10);
	//ros::Rate loop_rate(5);

	/* Subscribe to player topic */
	ros::NodeHandle video_listener_node;
	ros::Subscriber game_start_sub = video_listener_node.subscribe(String("Video") + to_string(player_id), 100, gameStartCallback);

    // Initialize the video 

    // Wait until the video is definitely ready to go

    // Create the video capture thread
    videoCaptureThread();

    ros::Rate loop_rate(30);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    // Shutdown the video thread cleanly
    // Close all files
    exit(0);
}

int videoCaptureThread(){
	
    CvCapture* capture = 0;
   	Mat frame, image;
 	char log_string[9];

	/* Logfile is open for append */ 
	FILE * pFile;
	char filename[]="logfile.txt"; /* Construct the filename */
	pFile = fopen (filename,"a");
	if (!pFile)
	{
  	  	printf(" ---  Couldn't create or open logfile :S  ----\n");
		return 1;
    }	
 	
	/* OpenCV connect camera*/

	
   		/* Receive video from webcam */
  	      capture = cvCaptureFromCAM(-1 );
 	




    
    /*	cvNamedWindow( "VideoCapture", 1 );******/






/* For each frame write to video each frame
    and to the logfile: FrameId, message, time, game#, block#, turn#, play#, value# */
   cout << "Got past initialization.  Entering capture loop." << endl;
   if( capture )
    {

	 IplImage* iplImg = cvQueryFrame( capture );
	 CvSize imgSize;
	 imgSize.width = iplImg->width;
 	 imgSize.height = iplImg->height;

	 double fps = cvGetCaptureProperty(capture,CV_CAP_PROP_FPS);
    	 CvVideoWriter *writer = cvCreateVideoWriter("/tmp/out.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps,imgSize); /*construct the filename for video*/
    cout << "Capturing" << endl;
	/* Publish Ready */
	/*if(ros::ok())
        {
 		std::stringstream ss;
 		ss << "Ready";
 		std_msgs::String msg;
 		msg.data = ss.str();
 		video.publish(msg);
 		ROS_INFO("I published [%s]", ss.str().c_str());
 		ros::spinOnce();
 		loop_rate.sleep();
       }
    */

        for(frame_id=1;;)
        {
            iplImg = cvQueryFrame( capture );
            frame = iplImg;

            if( frame.empty() )
                break;

	    /*log_string= "Frame: " + itoa(Id);	  */
	    strcpy(log_string, "Frame: \n");	
	    fputs (log_string,pFile);

	    /* Recording */
   	    if(iplImg==NULL)
        	printf("--- There's a problem with the video capture\n");
  	    else
	        cvWriteFrame(writer, iplImg);

    		
            if( waitKey( 10 ) >= 0 ) /*Check for a special key*/
                goto _cleanup_;
        }

        waitKey(0);
_cleanup_:
	/* Publish Finish */
/*
	if(ros::ok())
        {
 		std::stringstream ss;
 		ss << "Finish";
 		std_msgs::String msg;
 		msg.data = ss.str();
 		video.publish(msg);
 		ROS_INFO("I published [%s]", ss.str().c_str());
 		ros::spinOnce();
 		loop_rate.sleep();
       }
*/
	
        cvReleaseCapture( &capture );
        cvReleaseVideoWriter(&writer);

    }

    else
       printf("~~~ There is no capture ~~~\n");

/*     cvDestroyWindow("VideoCapture");*/
    fclose (pFile);
    return 0;
}




