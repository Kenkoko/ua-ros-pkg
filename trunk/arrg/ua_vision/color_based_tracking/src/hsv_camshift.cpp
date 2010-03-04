/*********************************************

A Ros node of the OpenCV camshift hsv-based histogram tracking demo.

Usage:

	rosrun color_based_tracking hsv_camshift /image_stream

Examples:

Terminal1:	roslaunch color_based_tracking wubble_test.launch
Terminal2:	rosrun color_based_tracking hsv_camshift overhead_cam/image_raw
	
	Then, in the CamshiftDemo window select an object with the
	mouse, and camshift will try to track it based on hue. Try
	both colored and white or gray objects. You can teleoperate the
	wubble using the WASD keys in the first terminal.
	
Written by: Jeremy Wright
with code pasted from camshift.cpp

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


IplImage *image, *hsv, *hue, *mask, *backproject, *histimg, *cv_image;
CvHistogram *hist;

int backproject_mode;
int select_object;
int track_object;
int show_hist;
CvPoint origin;
CvRect selection, box;
CvRect track_window;
CvBox2D track_box;
CvConnectedComp track_comp;
int hdims;
float hranges_arr[2];
float* hranges;
int vmin, vmax, smin;

IplImage* frame;
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

	image = 0;
	hsv = 0;
	hue = 0;
	mask = 0;
	backproject= 0;
	histimg = 0;
	hist = 0;
	frame = 0;
	cv_image = 0;

	backproject_mode = 0;
	select_object = 0;
	track_object = 0;
	show_hist = 1;
	hdims = 16;
	hranges_arr = {0,180};
	hranges = hranges_arr;
	vmin = 10;
	vmax = 256;
	smin = 140;
	cvNamedWindow( "Histogram", 1 );
	cvNamedWindow( "CamShiftDemo", 1 );
	cvSetMouseCallback( "CamShiftDemo", on_mouse, 0 );
//	cvCreateTrackbar( "Vmin", "CamShiftDemo", &vmin, 256, 0 );
//	cvCreateTrackbar( "Vmax", "CamShiftDemo", &vmax, 256, 0 );
//	cvCreateTrackbar( "Smin", "CamShiftDemo", &smin, 256, 0 );

	args = argv;
	image_sub_ = it_.subscribe(args[1], 1, &Tracker::imageCallback, this);

}

~Tracker()
{
	cvDestroyWindow("Image window");
}

CvScalar hsv2rgb( float hue )
{
    int rgb[3], p, sector;
    static const int sector_data[][3]=
        {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
    hue *= 0.033333333333333333333333333333333f;
    sector = cvFloor(hue);
    p = cvRound(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;

    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0],0);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{


	try
	{
		cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error");
	}




//	selection = cvRect(450,450,100,100);

        frame = cv_image;

        if( !image )
        {

            image = cvCreateImage( cvGetSize(frame), 8, 3 );
            image->origin = frame->origin;
            hsv = cvCreateImage( cvGetSize(frame), 8, 3 );
            hue = cvCreateImage( cvGetSize(frame), 8, 1 );
            mask = cvCreateImage( cvGetSize(frame), 8, 1 );
            backproject = cvCreateImage( cvGetSize(frame), 8, 1 );
            hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
            histimg = cvCreateImage( cvSize(320,200), 8, 3 );
            cvZero( histimg );
        }

        cvCopy( frame, image, 0 );
        cvCvtColor( image, hsv, CV_BGR2HSV );


        if( track_object )
        {
            int _vmin = vmin, _vmax = vmax;

            cvInRangeS( hsv, cvScalar(0,smin,MIN(_vmin,_vmax),0),
                        cvScalar(180,256,MAX(_vmin,_vmax),0), mask );
            cvSplit( hsv, hue, 0, 0, 0 );

            if( track_object < 0 )
            {
                float max_val = 0.f;
                cvSetImageROI( hue, selection );
                cvSetImageROI( mask, selection );
                cvCalcHist( &hue, hist, 0, mask );
                cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
                cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );
                cvResetImageROI( hue );
                cvResetImageROI( mask );
                track_window = selection;
                track_object = 1;

                cvZero( histimg );
                bin_w = histimg->width / hdims;
                for( i = 0; i < hdims; i++ )
                {
                    int val = cvRound( cvGetReal1D(hist->bins,i)*histimg->height/255 );
                    CvScalar color = hsv2rgb(i*180.f/hdims);
                    cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
                                 cvPoint((i+1)*bin_w,histimg->height - val),
                                 color, -1, 8, 0 );
                }
            }

            cvCalcBackProject( &hue, backproject, hist );
            cvAnd( backproject, mask, backproject, 0 );
            cvCamShift( backproject, track_window,
                        cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
                        &track_comp, &track_box );
            track_window = track_comp.rect;

            if( backproject_mode )
                cvCvtColor( backproject, image, CV_GRAY2BGR );
            if( !image->origin )
                track_box.angle = -track_box.angle;
            cvEllipseBox( image, track_box, CV_RGB(255,0,0), 3, CV_AA, 0 );
        }

        if( select_object && selection.width > 0 && selection.height > 0 )
        {
            cvSetImageROI( image, selection );
            cvXorS( image, cvScalarAll(255), image, 0 );
            cvResetImageROI( image );
        }

/**/

	cvShowImage( "CamShiftDemo", image );
	cvShowImage( "Histogram", histimg );
	cvWaitKey(3);
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
	Tracker ic(n,argv);
	ros::spin();
	return 0;
}


