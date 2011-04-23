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
#include "game_faces/VideoMsg.h"

// OpenCV and Boost
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <boost/thread/thread.hpp>

// C++ stuff
#include <cassert>
#include <iostream>
#include <fstream>

//#include <time.h>
#include <sys/time.h>

using namespace std;
using namespace boost::gregorian;
using namespace boost::posix_time;

// Flags for debugging
bool nocapture = false;   // Don't activate video camera; simulate 30fps on the frame_number

int done = false;
//FILE * pFile = 0;
int player_id;
int frame_number = 0;
ros::Subscriber game_sub;
ros::Publisher game_pub;

double starttime;

template <class T>
inline std::string to_string (const T& t)
{
    std::stringstream ss;
    ss << t;
    return ss.str();
}

void gamePlayStdin();

void frameRate() {
    int frames = 0;
    double framerate;
    while( !done ) {
        boost::this_thread::sleep(boost::posix_time::seconds(2));
        framerate = (frame_number - frames)/2.0;
        frames = frame_number;
        cout<< "Framerate: " << framerate << endl;
    }
}

void GetKeyFromCommandLine(){
    char key;
    while( !done ){
        cin >> key;
        if ( key == 'm' ) {
            cout << "recieved m" << endl;
            gamePlayStdin();
        }
        else if( key == 'q' || key == 'Q' ){
            done = true;
        }
    }
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
    int dropped = 0;

    {
        if (nocapture) {
            cout << "Running in simulated capture mode..." << endl;
            return;
        }
        
        /* OpenCV connect camera*/    	
        cout << "Getting Camera " << CV_CAP_ANY << endl; 	
        capture = cvCreateCameraCapture (CV_CAP_ANY);     // Use by default
        //capture = cvCreateCameraCapture (0);     // Use by default
        //capture = cvCreateCameraCapture (1);       // use for N's laptop
        //capture = cvCaptureFromCAM(CV_CAP_ANY);
        //capture = cvCaptureFromCAM(-1);
        //capture = cvCaptureFromCAM(1);    //TEMPORARY DEBUGGING
        
        // For each frame, write the frame to video and write to the logfile:
        //   FrameId, message, time, game#, block#, turn#, play#, value#

        if( !capture ){
            cout << "--- Could not start video capture" << endl;
        }
        else {
            cout << "Capturing" << endl;
            
            // This may depend on the supported resolutions of the camera; comment out to auto-config
            //cvQueryFrame( capture );  // Uncommenting this crashes the program, but is supposed to make it work
            cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 960 );
            cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 720 );
            
            int retries = 0;
            IplImage* frame = NULL;
            while (!frame && retries < 3) {
                frame = cvQueryFrame( capture );
                retries++;
            }
            if (retries == 3) {
                cout << "Querying first frame failed" << endl;
            }
            else {
            printf("frame : %dx%d\n", frame->width, frame->height);
            
            CvSize imgSize;
            imgSize.width = frame->width;   // These must be set to the resolution coming from the capture device.
            imgSize.height = frame->height;
            double fps = cvGetCaptureProperty(capture,CV_CAP_PROP_FPS);
            //double fps = 0;
            if (fps < 1){
                cout << "Could not get camera capture rate, so guessing 15fps." << endl;
                fps = 15;
            }
            
            char filetime[36];
            char filename[64];
	    char filehdr[64];
            time_t rawtime;
            time( &rawtime );
            struct tm* timenow = localtime( &rawtime );
            strftime(filetime,32,"%Y-%m-%d_%H:%M:%S", timenow);

            sprintf(filename,"videolog_p%d_%s.avi", player_id, filetime);
	    sprintf(filehdr,"videolog_p%d_%s", player_id, filetime);
            
            // supported codecs according to cap_gstreamer (line 449):
            // (run gst-inspect-0.10 to see if you have this codec installed)
            // CV_FOURCC('H','F','Y','U') => "ffenc_huffyuv";
            // CV_FOURCC('D','R','A','C') => "diracenc";
            // CV_FOURCC('X','V','I','D') => "xvidenc";
            // CV_FOURCC('X','2','6','4') => "x264enc";
            // CV_FOURCC('M','P','1','V') => "mpeg2enc";
            cout << imgSize.width << ' ' << imgSize.height << ' ' << fps << ' ' << filename << ' ' << CV_FOURCC('X', 'V', 'I', 'D') << endl;
            CvVideoWriter *writer = cvCreateVideoWriter(filename,
                                                        //CV_FOURCC('H', 'F', 'Y', 'U'),  // works on ubuntu hd (ffenc_huffyuv) (lossless)
                                                        //CV_FOURCC('M', 'P', '1', 'V'), // (mpeg2enc)
                                                        CV_FOURCC('X', 'V', 'I', 'D'),  // works on N's linux and ubuntu hd
                                                        //CV_FOURCC('M', 'J', 'P', 'G'),  // works on a mac
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

			// This saves the images to files
			//sprintf(filename, "%s_%d.jpg", filehdr, frame_number);
			//if (!cvSaveImage(filename, frame)) printf("Could not save: %s\n", filename);
                    }
                    //Check if any keys have been pressed, for quitting.
                    // Note:  This is only for the GUI interface, and we don't have one.
                    // However, without it, it appears to burn lots of CPU somewhere
                    //char key;
            		//key = cvWaitKey (wait_len);
            		//printf("%c", key);
            		//if( key == 'q' || key == 'Q' ){
                    //    done = true;
                    //}
                    // This version tries to burn less, but it doesn't seem to interact with OpenCV as expected

            	    boost::xtime_get(&xt2, boost::TIME_UTC);

                    add_time(xt, 1e9L/15L);     // This limits the video to 15 frames per second

                    boost::this_thread::sleep(xt);
                    if (time_diff(xt,xt2) < 0){
                    //    if (wait_len > 1)
                    //        --wait_len;
                        dropped ++;
                        //if (dropped % 10==0)
                            //cout << "Dropped "<<dropped<<" frames." << endl;
                    }
                    
                }
                cout << "Shutting down movie capture" << endl;
            }
            cvReleaseVideoWriter(&writer);
            cout << "Video writer released." << endl;
        }}
        cvReleaseCapture( &capture );
        cout << "Capture released." << endl;
    }
    //fclose (pFile);  
    done = true;

    cout << "Returning from video thread" << endl;
    return;
}

void gamePlayStdin() {
    int framenow = frame_number;    // Immediately get current frame number
    
    timeval mytimeval;
    int play_number;
    int amount;
    double msgtime;
    double mytime;
    double diff;
    
    gettimeofday(&mytimeval, NULL);
    mytime = mytimeval.tv_sec+mytimeval.tv_usec*0.0000001;  // Get my time to compare with time of message
    mytime -= starttime;   // Subtract out start time to get a more readable number
    
    cin >> play_number >> amount >> msgtime;
    
    msgtime -= starttime;   // Subtract out start time to get a more readable number
    diff = mytime - msgtime;
    
    //cout << "pn: " << play_number << " am: " << amount << " msgtime: " << msgtime << " mytime: " << mytime << endl;
    cout << "Message recieved, play: " << play_number << " Time diff: " << diff << endl;
        
    //char log_string[256];
    ROS_INFO("Received game play on stdin number: %d, amount: %d, frame_number: %d", 
              play_number, amount, frame_number);

    // Should send a message to the master with the frame number
    game_faces::VideoMsg vmsg;
    vmsg.frame_number = framenow;
    vmsg.player_id = player_id;
    vmsg.play_number = play_number;
    vmsg.amount = amount;
    game_pub.publish(vmsg);
    
    //sprintf(log_string, "%f\t%f\t%d\t%d\t%d\n", msgtime, mytime, framenow, play_number, amount);
    //fputs (log_string,pFile);
    //fflush(pFile);
}

// Obsolete with STDIN message passing
/*void gamePlayCallback(const game_faces::GamePlayConstPtr& msg)
{
    int framenow = frame_number;
    char log_string[256];
    ROS_INFO("Received game play number: %d, amount: %d,  frame_number: %d", 
              msg->play_number, msg->amount, framenow);

    // Should send a message to the master with the frame number
    game_faces::VideoMsg vmsg;
    vmsg.frame_number = framenow;
    vmsg.player_id = player_id;
    vmsg.play_number = msg->play_number;
    vmsg.amount = msg->amount;
    game_pub.publish(vmsg);
    
    //sprintf(log_string, "%d\t%d\t%d\n", framenow, msg->play_number, msg->amount);
    //fputs (log_string,pFile);
    //fflush(pFile);
}*/

void gameStartCallback(const game_faces::TwoPersonGameConstPtr& msg)
{
    ROS_INFO("Received game topic: [%s]", msg->game_topic.c_str());
    ros::NodeHandle n;
    //game_sub = n.subscribe(msg->game_topic, 100, gamePlayCallback);   // DEBUG: try to use stdin message passing
    string newtopic = to_string(msg->game_topic);
    newtopic.append("v");
    game_pub = n.advertise<game_faces::VideoMsg>(newtopic, 100);
}



int main( int argc,  char** argv )
{
    timeval startval;
    gettimeofday(&startval, NULL);
    starttime = startval.tv_sec + startval.tv_usec*0.0000001;

    if(argc < 2){
        cerr << "Error: Must pass in the player_id" << endl;
    }
    // Not very robust but just assume the first argument is the player_id
    player_id = atoi(argv[1]);
    
    // Will publish when the video is recording
    ros::init(argc, argv, to_string("Video") + to_string(player_id));
    //ros::NodeHandle n;
    //ros::Publisher video = n.advertise<std_msgs::String>("Video_signal", 10);
    //ros::Rate loop_rate(5);
    
    cout << "Host is: " << ros::network::getHost().c_str() << endl;

    /* Subscribe to player topic */
    ros::NodeHandle video_listener_node;
    ros::Subscriber game_start_sub = video_listener_node.subscribe(to_string("Video") + to_string(player_id), 100, gameStartCallback);

    
    // ** Logging should now be superfluous
    
    // Initialize the logfile
    //char filename[]="logfile.txt"; /* Construct the filename */
    
    /*char filetime[36];
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
        fputs("Frame\tPlay number\tAmount\n", pFile);
        fflush(pFile);
    }*/

    // Create the video capture thread
    boost::thread t1 = boost::thread(boost::bind(&VideoCaptureThread));
    
    // Create stdin listener thread
    boost::thread t2 = boost::thread::thread(boost::bind(&GetKeyFromCommandLine));
    
    // Create framerate thread
    boost::thread t3 = boost::thread::thread(boost::bind(&frameRate));

    ros::Rate loop_rate(30);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        if (nocapture)
            frame_number++;
    }
    // Shutdown the video thread cleanly
    done = true;
    
    cout << "Joining video thread" << endl;
    t1.join();
    
    //cout << "Joining listener thread" << endl;
    //t2.join();

    cout << "Ready to exit" << endl;

    exit(0);
}





