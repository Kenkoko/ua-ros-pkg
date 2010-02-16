/*********************************************

Creates node for single color tracking.

	single_color_tracking boolean-pub-image /image_stream red green blue

For example:

	rosrun wubble_vision single_color_tracking true /stereo/left/image_color 0 255 0

Should view through erratic's camera, and recognize green,
and output an image with that color outlined

Written by: Jeremy Wright
with code borrowed from various sources

*********************************************/

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "geometry_msgs/Polygon.h"
#include "wubble_vision/bounding_box.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <ctype.h>


IplImage *image, *mask, *backproject, *foreproject, *sum, *planes[3];
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
float range_arr[2], *ranges[3], maxlik, value, thepoint[2];
int rmin, gmin, bmin, rmax, gmax, bmax;

IplImage* frame,*r,*g,*b;
int i, bin_w, c;
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

Tracker(ros::NodeHandle &n, char** argv) :
	n_(n), it_(n_)
{

		frame = cvCreateImage( cvSize(1024,1024), 8, 3 );;
		image = cvCreateImage( cvGetSize(frame), 8, 3 );
		image->origin = frame->origin;
		mask = cvCreateImage( cvGetSize(frame), 8, 1 );
		backproject = cvCreateImage( cvGetSize(frame), 8, 1 );
		foreproject = cvCreateImage( cvGetSize(frame), 8, 1 );
		sum = cvCreateImage( cvSize(cvGetSize(frame).width+1,cvGetSize(frame).height+1), IPL_DEPTH_32S, 1 );
		bg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
		fg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
		lglikrat = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
		

	backproject_mode = 0;
	select_object = 0;
	track_object = 0;
	show_hist = 1;
	dims = {8,8,8};
	range_arr = {0,255};
	ranges = {range_arr,range_arr,range_arr};
/*	rmin = 0;
	gmin = 0;
	bmin = 0;
	rmax = 256;
	gmax = 256;
	bmax = 256;
/**/	cvNamedWindow( "Histogram", 1 );
	cvNamedWindow( "CamShiftDemo", 1 );
	cvSetMouseCallback( "CamShiftDemo", on_mouse, 0 );
/*	cvCreateTrackbar( "Rmin", "CamShiftDemo", &rmin, 256, 0 );
	cvCreateTrackbar( "Gmin", "CamShiftDemo", &gmin, 256, 0 );
	cvCreateTrackbar( "Bmin", "CamShiftDemo", &bmin, 256, 0 );
	cvCreateTrackbar( "Rmax", "CamShiftDemo", &rmax, 256, 0 );
	cvCreateTrackbar( "Gmax", "CamShiftDemo", &gmax, 256, 0 );
	cvCreateTrackbar( "Bmax", "CamShiftDemo", &bmax, 256, 0 );
/**/
	args = argv;
	show_image = (bool) (((std::string) args[1])=="true");
/*	std::string red = args[3];
	std::string green = args[4];
	std::string blue = args[5];
	bound_pub_ = n_.advertise<geometry_msgs::Polygon>("/color_tracking/boundbox_"+red+"_"+green+"_"+blue,1);
	if( show_image )
		image_pub_ = it_.advertise("/color_tracking/image_"+red+"_"+green+"_"+blue,1);
/**/
//	cvNamedWindow("Image window");
	image_sub_ = it_.subscribe(args[2], 1, &Tracker::imageCallback, this);

}

~Tracker()
{
	cvDestroyWindow("Image window");
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




//	box = cvRect(580,165,620-580,220-165); //grey box
//	box = cvRect(788,106,855-788,177-106); //gold box
//	box = cvRect(393,138,440-393,201-138); //white box

//	selection = cvRect(450,450,100,100);

//	frame = cv_image;
//	if( !frame )
//		break;

	if( !image )
	{

	}

	cvCopy( frame, image, 0 );

	if( track_object )
	{
		int _rmin = rmin, _gmin = gmin, _bmin = bmin;

//		cvInRangeS( image, cvScalar(_rmin,_gmin,_bmin,0), cvScalar(256,256,256,0), mask );
		r = cvCreateImage( cvGetSize(frame), 8, 1 );
		g = cvCreateImage( cvGetSize(frame), 8, 1 );
		b = cvCreateImage( cvGetSize(frame), 8, 1 );
		cvSplit( frame, r, g, b, 0 );
		planes = {r,g,b};

		if( track_object < 0 )
		{
			cvZero(mask);
	//		cvNot(mask,mask);
			cvRectangle(mask,cvPoint(selection.x,selection.y),cvPoint(selection.x+selection.width,selection.y+selection.height),cvScalarAll(255),CV_FILLED,8,0);
			float max_val = 0.f;
	/*		cvSetImageROI( mask, selection );
			for( int i = 0; i < 3; i++) {
				cvSetImageROI( planes[i], selection );}
	/**/		cvCalcHist( planes, fg_hist, 0, mask );
			cvGetMinMaxHistValue( fg_hist, 0, &max_val, 0, 0 );
			cvConvertScale( fg_hist->bins, fg_hist->bins, max_val ? 255. / max_val : 0., 0 );
			cvNot(mask,mask);
	/**/		cvCalcHist( planes, bg_hist, 0, mask );
			cvGetMinMaxHistValue( bg_hist, 0, &max_val, 0, 0 );
			cvConvertScale( bg_hist->bins, bg_hist->bins, max_val ? 255. / max_val : 0., 0 );
	/*		for( int i = 0; i < 3; i++) {
				cvResetImageROI( planes[i] );}
			cvResetImageROI( mask );
	/**/		track_window = selection;
			track_object = 1;
		}

		cvNormalizeHist(fg_hist, 1.0);
		cvNormalizeHist(bg_hist, 1.0);
		for( int m=0; m<8; m++){
			for( int n=0; n<8; n++){
				for( int o=0; o<8; o++){
					*(cvGetHistValue_3D(lglikrat,m,n,o)) = log(cvQueryHistValue_3D(fg_hist,m,n,o)/cvQueryHistValue_3D(bg_hist,m,n,o));
		}}}
		
		cvNormalizeHist(lglikrat,255);
		
		cvCalcBackProject( planes, backproject, lglikrat );
//		cvCalcBackProject( planes, foreproject, fg_hist );
//		cvNot(backproject, backproject);
//		cvAnd(foreproject, backproject, backproject, 0);
//		cvIntegral(backproject, sum, 0, 0);
/*
		maxlik = 0.0;
		value = 0.0;
		thepoint = {0,0};	
		for (int i=0; i<h-box.height; i++) {
			for (int j=0; j<w-box.width; j++) {
				value = cvGet2d(sum,i,j)+cvGet2d(sum,i+box.height,j+box.width)-cvGet2d(sum,i+box.height,j)-cvGet2d(sum,i,j+box.width);
	//			printf("%f, %f\n", value, maxlik);
				if (value > maxlik){
	//				printf("%f, %f\n", value, maxlik);
					maxlik = value;
					thepoint = {j,i};
				}
			}
		}
/**/
		cvCamShift( backproject, track_window,
			    cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
			    &track_comp, &track_box );
		track_window = track_comp.rect;

		if( backproject_mode )
			cvCvtColor( backproject, image, CV_GRAY2BGR );
		if( !image->origin )
			track_box.angle = -track_box.angle;
		cvEllipseBox( image, track_box, CV_RGB(255,0,0), 3, CV_AA, 0 );
/**/
	}

	if( select_object && selection.width > 0 && selection.height > 0 )
	{
		cvSetImageROI( image, selection );
		cvXorS( image, cvScalarAll(255), image, 0 );
		cvResetImageROI( image );
	}
/**/

	if( show_image ){
		cvShowImage( "CamShiftDemo", image );
		cvShowImage( "Histogram", backproject );
		cvWaitKey(10);
	}

/*	try
	{
		bound_pub_.publish(boxes);
		if( show_image ){
			image_pub_.publish(bridge_.cvToImgMsg(image, "bgr8")); }
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error");
	}
/**/
//	cvReleaseImage(&frame);
	cvReleaseImage(&image);
	cvReleaseImage(&mask);
	cvReleaseImage(&backproject);
	cvReleaseImage(&foreproject);
	cvReleaseImage(&r);
	cvReleaseImage(&g);
	cvReleaseImage(&b);
/**/

}

protected:

ros::NodeHandle n_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
sensor_msgs::CvBridge bridge_;
image_transport::Publisher image_pub_;
ros::Publisher bound_pub_;
char** args;
bool show_image;
geometry_msgs::Polygon boxes;

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tracker");
	ros::NodeHandle n;
	Tracker ic(n,argv);
	ros::spin();
	return 0;
}


