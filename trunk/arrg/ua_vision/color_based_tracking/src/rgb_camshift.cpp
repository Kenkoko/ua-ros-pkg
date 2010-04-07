/*********************************************


A Ros node with an RGB version of camshift. Can also accept
pre-computed histograms.

Usage:

	rosrun color_based_tracking rgb_camshift /image_stream [object_histogram]

Examples:

Terminal1:	roslaunch color_based_tracking teleop_wubble.launch
Terminal2:	rosrun color_based_tracking rgb_camshift overhead_cam/image_raw
	
	Then, in the Demo window select an object with the
	mouse, and camshift will try to track it. You can teleop the wubble
	using the WASD keys in the first terminal window.

	Alternatively, to use a precomputed histogram:
	
Terminal1:	roslaunch color_based_tracking teleop_wubble.launch
Terminal2:	roscd color_based_tracking
		rosrun color_based_tracking rgb_camshift overhead_cam/image_raw histograms/red_column_hist

Written by: Jeremy Wright
with code altered from camshift.cpp

*********************************************/

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <ctype.h>


IplImage *image, *frame, *mask, *backproject, *r, *g, *b, *planes[3];
CvHistogram *bg_hist, *fg_hist, *lglikrat;

int backproject_mode;
int select_object;
int track_object;
int show_hist;
CvPoint origin;
CvRect selection, box;
CvRect track_window;
CvBox2D track_box;
CvConnectedComp track_comp;
int dims[3];
float range_arr[2], *ranges[3], max_val, value, maxlik, epsilon;
int thepoint[2];
double *sumdata;
int h, w, step, boxhstep;

char* object;
FILE *histogram;
/**/

void on_mouse( int event, int x, int y, int flags, void* param )
{
	if( !image )
	return;

	if( image->origin )
	y = image->height - y;

	if( select_object )
	{
	selection.x = MIN(x,origin.x);
	selection.y = MIN(y,origin.y);
	selection.width = selection.x + CV_IABS(x - origin.x);
	selection.height = selection.y + CV_IABS(y - origin.y);

	selection.x = MAX( selection.x, 0 );
	selection.y = MAX( selection.y, 0 );
	selection.width = MIN( selection.width, image->width );
	selection.height = MIN( selection.height, image->height );
	selection.width -= selection.x;
	selection.height -= selection.y;
	}

	switch( event )
	{
	case CV_EVENT_LBUTTONDOWN:
	origin = cvPoint(x,y);
	selection = cvRect(x,y,0,0);
	select_object = 1;
	break;
	case CV_EVENT_LBUTTONUP:
	select_object = 0;
	if( selection.width > 0 && selection.height > 0 )
		track_object = -1;
	break;
	}
}


class Tracker {

public:

Tracker(ros::NodeHandle &n, int argc, char** argv) :
	n_(n), it_(n_)
{
//	box = cvRect(580,165,620-580,220-165); //grey box
//	box = cvRect(788,106,855-788,177-106); //gold box
//	box = cvRect(393,138,440-393,201-138); //white box

	select_object = 0;
	track_object = 0;
	show_hist = 1;
	dims = {8,8,8};
	range_arr = {0,255};
	ranges = {range_arr,range_arr,range_arr};
	epsilon = 0.1;
	fg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );

/**/	cvNamedWindow( "Extra", 1 );
	cvNamedWindow( "Demo", 1 );
	cvSetMouseCallback( "Demo", on_mouse, 0 );

	object = 0;
	if( argc > 2 ){
		object = argv[2];
		selection = cvRect(0,0,1023,1023);
		track_object = -1;

		histogram = fopen(object,"rb");
		fread(cvGetHistValue_3D(fg_hist,0,0,0),sizeof(float),dims[0]*dims[1]*dims[2],histogram);
		fclose(histogram);
	}
	image_sub_ = it_.subscribe(argv[1], 1, &Tracker::imageCallback, this);
}

~Tracker()
{
	cvDestroyWindow("Demo");
	cvDestroyWindow("Extra");
	if( !histogram ){	
		fclose(histogram);}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{

	try
	{
		frame = bridge_.imgMsgToCv(msg_ptr, "bgr8");
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error");
	}

	if( !image )
	{
		image = cvCreateImage( cvGetSize(frame), 8, 3 );
		image->origin = frame->origin;
		mask = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );
		backproject = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );
		bg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
		lglikrat = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
		r = cvCreateImage( cvGetSize(frame), 8, 1 );
		g = cvCreateImage( cvGetSize(frame), 8, 1 );
		b = cvCreateImage( cvGetSize(frame), 8, 1 );
	}

	cvCopy( frame, image, 0 );

	if( track_object )
	{
		cvSplit( image, r, g, b, 0 );
		planes = {r,g,b};

		if( track_object < 0 )
		{
			cvZero(mask);
			max_val = 0.f;
			if( !object ){
				cvRectangle(mask,cvPoint(selection.x,selection.y),cvPoint(selection.x+selection.width,selection.y+selection.height),cvScalarAll(255),CV_FILLED,8,0);
				cvCalcHist( planes, fg_hist, 0, mask);
			}
			cvNot(mask,mask);
			cvCalcHist( planes, bg_hist, 0, mask );
			cvNot(mask,mask);
			cvGetMinMaxHistValue( fg_hist, 0, &max_val, 0, 0 );
			cvNormalizeHist(bg_hist, max_val);
			for( int m=0; m<8; m++){
			for( int n=0; n<8; n++){
			for( int o=0; o<8; o++){
				*(cvGetHistValue_3D(lglikrat,m,n,o)) = log((cvQueryHistValue_3D(fg_hist,m,n,o)+epsilon)/(cvQueryHistValue_3D(bg_hist,m,n,o)+epsilon));
			}}}
/**/			track_window = selection;
			track_object = 1;
		}
//		cvNormalizeHist(lglikrat,255);
		cvCalcBackProject( planes, backproject, lglikrat );
		
//		cvAnd( backproject, mask, backproject, 0 );
		cvCamShift( backproject, track_window,
			cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
			&track_comp, &track_box );
		track_window = track_comp.rect;

		if( backproject_mode )
			cvCvtColor( backproject, image, CV_GRAY2BGR );
		if( !image->origin )
			track_box.angle = -track_box.angle;
		cvEllipseBox( image, track_box, CV_RGB(255,0,0), 3, CV_AA, 0 );

/**/	}

	if( select_object && selection.width > 0 && selection.height > 0 )
	{
		cvSetImageROI( image, selection );
		cvXorS( image, cvScalarAll(255), image, 0 );
		cvResetImageROI( image );
	}
/**/

	cvShowImage( "Demo", image );
	cvShowImage( "Extra", backproject);
	cvWaitKey(10);

}

protected:

ros::NodeHandle n_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
sensor_msgs::CvBridge bridge_;
image_transport::Publisher image_pub_;
char** args;

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tracker");
	ros::NodeHandle n;
	Tracker ic(n,argc,argv);
	ros::spin();
	return 0;
}


