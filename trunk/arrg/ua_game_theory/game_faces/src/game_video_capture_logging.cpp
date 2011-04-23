// University of Arizona
// game_faces
// Reads the messages from the game
// and creates the video and logfile

// THIS PROGRAM IS RUN JUST ONCE DURING THE WHOLE SESSION
// IT IS NOT CLOSED UNTIL THE EXPERIMENT IS DONE

// ROS stuff
#include <ros/ros.h>
#include "ros/network.h"
#include <std_msgs/String.h>
#include "game_faces/TwoPersonGame.h"
#include "game_faces/GamePlay.h"

// OpenCV and Boost
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <boost/thread/thread.hpp>

// C++ stuff
#include <cassert>
#include <iostream>
#include <fstream>

#include <time.h>

using namespace std;
using namespace std_msgs;
using namespace boost::gregorian;
using namespace boost::posix_time;

int done = false;
FILE * pFile = 0;
int frame_number = 0;
ros::Subscriber game_sub;

template <class T>
inline std::string to_string (const T& t)
{
	std::stringstream ss;
	ss << t;
	return ss.str();
}

// Add time in nanoseconds
void add_time(boost::xtime &xt, long dur){
    if(xt.nsec > 1e9-dur){
        xt.sec += 1;
        xt.nsec = xt.nsec+dur - 1e9;
    } else {
        xt.nsec += dur;                        
    }
}

long time_diff(boost::xtime &xt2, boost::xtime &xt1){
    return (xt2.sec-xt1.sec) * 1e9 + (xt2.nsec-xt1.nsec);
}

void VideoCaptureThread(){
	CvCapture* capture = NULL; 
    int wait_len = 10; // wait 10ms for a key...?  boo.

    {
        /* OpenCV connect camera*/    	
        //cout << "Getting Camera " << CV_CAP_ANY << endl; 	
        capture = cvCreateCameraCapture (CV_CAP_ANY);
        //capture = cvCaptureFromCAM(-1);
        
        // For each frame, write the frame to video and write to the logfile:
        //   FrameId, message, time, game#, block#, turn#, play#, value#

        if( !capture ){
            printf("--- Could not start video capture\n");
        }
        else {
            cout << "Capturing" << endl;
            IplImage* frame = cvQueryFrame( capture );
            CvSize imgSize;
            imgSize.width = frame->width;
            imgSize.height = frame->height;
            double fps = cvGetCaptureProperty(capture,CV_CAP_PROP_FPS);
            if (fps < 1){
                cout << "Could not get camera capture rate, so guessing 15fps." << endl;
                fps = 15;
            }
            CvVideoWriter *writer = cvCreateVideoWriter("/tmp/out.avi", 
                                                        CV_FOURCC('M', 'J', 'P', 'G'), 
                                                        fps,imgSize);
            if( !writer ){
                printf("--- Could not start video writing\n");                
            } else {                
                cout << "Writing movie file." << endl;
                boost::xtime xt, xt2;
                while( !done )
                {
            	    boost::xtime_get(&xt, boost::TIME_UTC);
                    frame = cvQueryFrame(capture);
                    /* Recording */
                    if(frame==NULL)
                    {
                        printf("--- There's a problem with the video capture\n");
                        break;
                    }
                    else
                    {
                        frame_number++;
                        cvWriteFrame(writer, frame);
                    }
                    //Check if any keys have been pressed, for quitting.
                    // Note:  This is only for the GUI interface, and we don't have one.
                    // However, without it, it appears to burn lots of CPU somewhere
            		//key = cvWaitKey (wait_len);
            		//if( key == 'q' || key == 'Q' ){
                    //    done = true;
                    //}
                    // This version tries to burn less, but it doesn't seem to interact with OpenCV as expected
                    //cout << "Time started loop:   " << xt.sec<< "." << xt.nsec/1.0e9 << endl;
            	    boost::xtime_get(&xt2, boost::TIME_UTC);
                    //cout << "Time now:            " << xt2.sec<< "." << xt2.nsec/1.0e9 << endl;
                    add_time(xt, 1e9L/15L);
                    //cout << "Time to sleep until: " << xt.sec<< "." << xt.nsec/1.0e9 << ", diff: "<< time_diff(xt,xt2) << endl;
                    boost::this_thread::sleep(xt);
                    if (time_diff(xt,xt2) < 0){
                    //    if (wait_len > 1)
                    //        --wait_len;
                        cout << "Dropped frame." << endl;
                    }
                }
                cout << "Shutting down movie capture" << endl;
            }
            cvReleaseVideoWriter(&writer);
        }
        cvReleaseCapture( &capture );
    }
    //fclose (pFile);
    //done = true;
    return;
}

void gamePlayCallback(const game_faces::GamePlayConstPtr& msg)
{
    char log_string[256];
    ROS_INFO("Received game play number: %d, amount: %d,  frame_number: %d", 
              msg->play_number, msg->amount, frame_number);

    sprintf(log_string, "New Message at Frame: %d  -  game play number: %d, amount: %d\n", frame_number, msg->play_number, msg->amount);
    fputs (log_string,pFile);
    fflush(pFile);
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
    ros::init(argc, argv, to_string("Video") + to_string(player_id));
    //ros::init(argc, argv, String("Video") + to_string(player_id)); // ORIGINAL
    //ros::NodeHandle n;
    //ros::Publisher video = n.advertise<std_msgs::String>("Video_signal", 10);
    //ros::Rate loop_rate(5);
    
    cout << "Host is: " << ros::network::getHost().c_str() << endl;

    /* Subscribe to player topic */
    ros::NodeHandle video_listener_node;
    //ros::Subscriber game_start_sub = video_listener_node.subscribe(String("Video") + to_string(player_id), 100, gameStartCallback);
    ros::Subscriber game_start_sub = video_listener_node.subscribe(to_string("Video") + to_string(player_id), 100, gameStartCallback);

    // Initialize the logfile


    //char filename[]="videolog.txt"; /* Construct the filename */
    char filetime[36];
    char filename[64];
    time_t rawtime;
    time( &rawtime );
    struct tm* timenow = localtime( &rawtime );
    strftime(filetime,32,"%Y-%m-%d_%H:%M:%S", timenow);

    sprintf(filename,"videolog_p%d_%s.txt", player_id, filetime);

    pFile = fopen (filename,"w");
    if (!pFile)
    {
        printf(" ---  Couldn't create or open logfile :S  ----\n");
        exit(1);
    }
    else {
        printf(" Logging to %s\n", filename);
    }

    // Create the video capture thread
    boost::thread t1 = boost::thread(boost::bind(&VideoCaptureThread)); // Changed: was boost::thread::thread
    

    ros::Rate loop_rate(30);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    // Shutdown the video thread cleanly
    done = true;
    t1.join();

    exit(0);
}





