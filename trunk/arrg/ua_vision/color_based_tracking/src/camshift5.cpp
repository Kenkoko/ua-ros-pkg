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
//#include "wubble_vision/bounding_box.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <ctype.h>


IplImage *image, *mask, *backproject, *foreproject, *sum, *like, *planes[3];
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
float range_arr[2], *ranges[3];
int value, maxlik, thepoint[2];

IplImage* frame,*r,*g,*b;
int i, bin_w, c;

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

Tracker(ros::NodeHandle &n, char** argv) :
	n_(n), it_(n_)
{
printf("Test 1");

//	box = cvRect(580,165,620-580,220-165); //grey box
	box = cvRect(788,106,855-788,177-106); //gold box
//	box = cvRect(393,138,440-393,201-138); //white box

	backproject_mode = 0;
	select_object = 0;
	track_object = -1;
	show_hist = 1;
	dims = {8,8,8};
	range_arr = {0,255};
	ranges = {range_arr,range_arr,range_arr};

/**/	cvNamedWindow( "Histogram", 1 );
	cvNamedWindow( "CamShiftDemo", 1 );
	cvSetMouseCallback( "CamShiftDemo", on_mouse, 0 );

	args = argv;
	show_image = (bool) (((std::string) args[1])=="true");
/*	std::string red = args[3];
	std::string green = args[4];
	std::string blue = args[5];
	bound_pub_ = n_.advertise<geometry_msgs::Polygon>("/color_tracking/boundbox_"+red+"_"+green+"_"+blue,1);
	if( show_image )
*/		image_pub_ = it_.advertise("/color_tracking/lglikrat",1);
/**/

	selection = cvRect(0,0,1023,1023);

	fg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
	object = args[3];
	histogram = fopen(object,"rb");
	fread(cvGetHistValue_3D(fg_hist,0,0,0),sizeof(float),dims[0]*dims[1]*dims[2],histogram);
/**/	fclose(histogram);
	image_sub_ = it_.subscribe(args[2], 1, &Tracker::imageCallback, this);

printf("Test 2");
}

~Tracker()
{
	cvDestroyWindow("Image window");
	cvDestroyWindow("Histogram");
	if( !histogram ){	
		fclose(histogram);}
}

void myCalcBackProject( CvArr** img, CvArr* dst, const CvHistogram* hist )
{
    if( !CV_IS_HIST(hist))
        CV_Error( CV_StsBadArg, "Bad histogram pointer" );

    if( !img )
        CV_Error( CV_StsNullPtr, "Null double array pointer" );

    int size[CV_MAX_DIM];
    int i, dims = cvGetDims( hist->bins, size );
    
    bool uniform = CV_IS_UNIFORM_HIST(hist);
    const float* uranges[CV_MAX_DIM] = {0};
    const float** ranges = 0;
    
    if( hist->type & CV_HIST_RANGES_FLAG )
    {
        ranges = (const float**)hist->thresh2;
        if( uniform )
        {
            for( i = 0; i < dims; i++ )
                uranges[i] = &hist->thresh[i][0];
            ranges = uranges;
        }
    }
    
    cv::vector<cv::Mat> images(dims);
    for( i = 0; i < dims; i++ )
        images[i] = cv::cvarrToMat(img[i]);
    
    cv::Mat _dst = cv::cvarrToMat(dst);
    
    CV_Assert( _dst.size() == images[0].size() && _dst.depth() == images[0].depth() );
    
    if( !CV_IS_SPARSE_HIST(hist) )
    {
        cv::MatND H((const CvMatND*)hist->bins);
        cv::calcBackProject( &images[0], (int)images.size(),
                            0, H, _dst, ranges, 1, uniform );
    }
    else
    {
        cv::SparseMat sH((const CvSparseMat*)hist->bins);
        cv::calcBackProject( &images[0], (int)images.size(),
                             0, sH, _dst, ranges, 1, uniform );
    }
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

	box = selection;
//	selection = cvRect(450,450,100,100);

	if( !image )
	{
		image = cvCreateImage( cvGetSize(frame), 8, 3 );
		image->origin = frame->origin;
		backproject = cvCreateImage( cvGetSize(frame), 8, 1 );
		foreproject = cvCreateImage( cvGetSize(frame), 8, 1 );
		mask = cvCreateImage( cvGetSize(frame), 8, 1 );
		sum = cvCreateImage( cvSize(cvGetSize(frame).width+1,cvGetSize(frame).height+1), IPL_DEPTH_32S, 1 );
		like = cvCreateImage( cvSize(cvGetSize(frame).width+1,cvGetSize(frame).height+1), IPL_DEPTH_32S, 1 );
		bg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
//		fg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
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
			float max_val = 0.f;
//			cvCalcHist( planes, fg_hist, 0, mask );
/*			cvGetMinMaxHistValue( fg_hist, 0, &max_val, 0, 0 );
			cvConvertScale( fg_hist->bins, fg_hist->bins, max_val ? 255. / max_val : 0., 0 );
/**/			cvNot(mask,mask);
			cvCalcHist( planes, bg_hist, 0, mask );
			cvNot(mask,mask);
/*			cvGetMinMaxHistValue( bg_hist, 0, &max_val, 0, 0 );
			cvConvertScale( bg_hist->bins, bg_hist->bins, max_val ? 255. / max_val : 0., 0 );
/**/			track_window = selection;
			track_object = 1;
		}

		cvNormalizeHist(fg_hist, 1.0);
		cvNormalizeHist(bg_hist, 1.0);
		for( int m=0; m<8; m++){
			for( int n=0; n<8; n++){
				for( int o=0; o<8; o++){
					*(cvGetHistValue_3D(lglikrat,m,n,o)) = log(cvQueryHistValue_3D(fg_hist,m,n,o)/cvQueryHistValue_3D(bg_hist,m,n,o));
		}}}
	
//		cvNormalizeHist(lglikrat,255);
//		cvCalcBackProject( planes, backproject, lglikrat );
		myCalcBackProject( (CvArr**) planes, (CvArr*) backproject, lglikrat );
		cvIntegral(backproject, sum, 0, 0);

		maxlik = 0;
		value = 0;
		thepoint = {0,0};	
		for (int i=0; i<(sum->height-(box.height+1)); i++) {
			for (int j=0; j<(sum->width-(box.width+1)); j++) {
				value += ((int *)(sum->imageData + i*sum->widthStep))[j];
				value += ((int *)(sum->imageData + (i+(box.height+1))*sum->widthStep))[(j+(box.width+1))];
				value -= ((int *)(sum->imageData + (i+(box.height+1))*sum->widthStep))[j];
				value -= ((int *)(sum->imageData + i*sum->widthStep))[(j+(box.width+1))];
				cvSet2D(like,i,j,cvScalar(value));
	//			printf("%f, %f\n", value, maxlik);
				if (value > maxlik){
	//				printf("%f, %f\n", value, maxlik);
					maxlik = value;
					thepoint = {j,i};
				}
			}
		}

		cvRectangle(image,cvPoint(box.x,box.y),cvPoint(box.x+box.width,box.y+box.height),cvScalar(0,0,255),1,8,0);
		cvRectangle(image,cvPoint(thepoint[0],thepoint[1]),cvPoint(thepoint[0]+box.width,thepoint[1]+box.height),cvScalarAll(255),1,8,0);	
/**/
/*		cvCamShift( backproject, track_window,
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
		cvShowImage( "Histogram", mask );
		cvWaitKey(10);
	}

/*	try
	{
		//bound_pub_.publish(boxes);
		image_pub_.publish(bridge_.cvToImgMsg(backproject, "mono8"));
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error");
	}
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


