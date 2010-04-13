// University of Arizona
// game_faces
// Reads the messages from the game
// and creates the video and logfile

// THIS PROGRAM IS RUN JUST ONCE DURING THE WHOSE SESSION
// IT IS NOT CLOSED UNTIL THE EXPERIMENT IS DONE

#define CV_NO_BACKWARD_COMPATIBILITY

#include "cv.h"
#include "highgui.h"

#include <ros/ros.h>

#include "/opt/ros/installed/ros/std_msgs/msg/cpp/std_msgs/String.h"
#include <sstream>

#include <iostream>
#include <cstdio>
#include <string>

#ifdef _EiC
#define WIN32
#endif

using namespace std;
using namespace cv;

int Id=0;
String Logmsg; 

template <class T>
inline std::string to_string (const T& t)
{
	std::stringstream ss;
	ss << t;
	return ss.str();
}

void chatterCallback(const std_msgs::StringConstPtr& msg)
{
  ROS_INFO("Received [%s]", msg->data.c_str());
  String strId;
  strId=to_string(Id) ; /* Construct the log message  */
  Logmsg.assign(strId + " " + msg->data.c_str());
}



int main( int argc,  char** argv )
{

	/* Will publish when the video is recording */
	ros::init(argc, argv, "Video");
	ros::NodeHandle n;
	ros::Publisher video = n.advertise<std_msgs::String>("Video_signal", 10);
	ros::Rate loop_rate(5);

	/* Subscribe to player topic */
	ros::init(argc, argv, "video_listener");
	ros::NodeHandle nl;
	ros::Subscriber chatter_sub = nl.subscribe("player_topic", 100, chatterCallback);
	ros::spin();


	int i = 1;
	
    	CvCapture* capture = 0;
   	Mat frame, image;
    	String inputName;
    	inputName.assign( argv[i] );
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

	if( inputName.empty() || (isdigit(inputName.c_str()[0]) && inputName.c_str()[1] == '\0') )
 	{
   		/* Receive video from webcam */
  	      capture = cvCaptureFromCAM( inputName.empty() ? 0 : inputName.c_str()[0] - '0' );
 	 }
  	  else 
   	 {
		printf(" ---  Receive video from file %s  ----\n", inputName.c_str());
		/* Receive video from file */
		capture = cvCaptureFromAVI( inputName.c_str() );
           	 
   	 }





    
    /*	cvNamedWindow( "VideoCapture", 1 );******/






/* For each frame write to video each frame
    and to the logfile: FrameId, message, time, game#, block#, turn#, play#, value# */
   	
   if( capture )
    {

	 IplImage* iplImg = cvQueryFrame( capture );
	 CvSize imgSize;
	 imgSize.width = iplImg->width;
 	 imgSize.height = iplImg->height;

	 double fps = cvGetCaptureProperty(capture,CV_CAP_PROP_FPS);
    	 CvVideoWriter *writer = cvCreateVideoWriter("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps,imgSize); /*construct the filename for video*/

	/* Publish Ready */
	if(ros::ok())
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


        for(Id=1;;)
        {
            iplImg = cvQueryFrame( capture );
            frame = iplImg;

            if( frame.empty() )
                break;

	    /*log_string= "Frame: " + itoa(Id);	  */
	    ros::spinOnce();
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

	
        cvReleaseCapture( &capture );
        cvReleaseVideoWriter(&writer);

    }

    else
       printf("~~~ There is no capture ~~~\n");

/*     cvDestroyWindow("VideoCapture");*/
    fclose (pFile);
    return 0;
}












/****************************************/

/*
    
    CvCapture* input = cvCaptureFromFile(argv[1]);
    IplImage* image = cvRetrieveFrame(input);

    if (!image) {
        printf("Unable to read input");
        return 0;
    }


    CvSize imgSize;
    imgSize.width = image->width;
    imgSize.height = image->height;



    double fps = cvGetCaptureProperty(
            input,
            CV_CAP_PROP_FPS
            );

    CvVideoWriter *writer = cvCreateVideoWriter(
            "out.avi",
            CV_FOURCC('M', 'J', 'P', 'G'),
            fps,
            imgSize
            );


    IplImage* colourImage;
    //Keep processing frames...
    for (;;) {

        //Get a frame from the input video.
        colourImage = cvQueryFrame(input);
        if(colourImage==NULL)
        	fprintf("There's a problem with the video capture\n");
	else
	        cvWriteFrame(writer, colourImage);

    }

    cvReleaseVideoWriter(&writer);
    cvReleaseCapture(&input);

}

















  /*  CascadeClassifier cascade, nestedCascade;
    double scale = 1;

    for( int i = 1; i < argc; i++ )
    {
        if( cascadeOpt.compare( 0, cascadeOptLen, argv[i], cascadeOptLen ) == 0 )
            cascadeName.assign( argv[i] + cascadeOptLen );
        else if( nestedCascadeOpt.compare( 0, nestedCascadeOptLen, argv[i], nestedCascadeOptLen ) == 0 )
        {
            if( argv[i][nestedCascadeOpt.length()] == '=' )
                nestedCascadeName.assign( argv[i] + nestedCascadeOpt.length() + 1 );
            if( !nestedCascade.load( nestedCascadeName ) )
                cerr << "WARNING: Could not load classifier cascade for nested objects" << endl;
        }
        else if( scaleOpt.compare( 0, scaleOptLen, argv[i], scaleOptLen ) == 0 )
        {
            if( !sscanf( argv[i] + scaleOpt.length(), "%lf", &scale ) || scale < 1 )
                scale = 1;
        }
        else if( argv[i][0] == '-' )
        {
            cerr << "WARNING: Unknown option %s" << argv[i] << endl;
        }
        else
            inputName.assign( argv[i] );
    }

    if( !cascade.load( cascadeName ) )
    {
        cerr << "ERROR: Could not load classifier cascade" << endl;
        cerr << "Usage: facedetect [--cascade=\"<cascade_path>\"]\n"
            "   [--nested-cascade[=\"nested_cascade_path\"]]\n"
            "   [--scale[=<image scale>\n"
            "   [filename|camera_index]\n" ;
        return -1;
    }

    if( inputName.empty() || (isdigit(inputName.c_str()[0]) && inputName.c_str()[1] == '\0') )
        capture = cvCaptureFromCAM( inputName.empty() ? 0 : inputName.c_str()[0] - '0' );
    else if( inputName.size() )
    {
        image = imread( inputName, 1 );
        if( image.empty() )
            capture = cvCaptureFromAVI( inputName.c_str() );
    }
    
    cvNamedWindow( "result", 1 );

    if( capture )
    {
        for(;;)
        {
            IplImage* iplImg = cvQueryFrame( capture );
            frame = iplImg;
            if( frame.empty() )
                break;
            if( iplImg->origin == IPL_ORIGIN_TL )
                frame.copyTo( frameCopy );
            else
                flip( frame, frameCopy, 0 );
	   
            cv::imshow( "result", frame );  


_cleanup_:
        cvReleaseCapture( &capture );
    }
  
   

    cvDestroyWindow("result");

    return 0;
}

*/





