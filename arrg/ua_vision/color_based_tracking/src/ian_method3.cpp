/*********************************************

try:

	rosrun color_based_tracking camshift5 true overhead_cam/image_raw histograms/white_block_hist

Written by: Jeremy Wright
with code borrowed from various sources

*********************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
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
#include <boost/array.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/blas.hpp>

namespace ublas = boost::numeric::ublas;

ublas::matrix<double> K (3, 3);

class Tracker {

public:

Tracker(ros::NodeHandle &n, int argc, char** argv) :
	n_(n), it_(n_)
{
	track_object = 0;
	dims[0] = 8; dims[1] = 8; dims[2] = 8;
	range_arr[0] = 0; range_arr[1] = 255;
	ranges[0] = range_arr; ranges[1] = range_arr; ranges[2] = range_arr;
	epsilon = 0.1;
	fg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
	storage = cvCreateMemStorage(0);
	contour = 0;

/**/	cvNamedWindow( "Extra", 1 );
	cvNamedWindow( "Demo", 1 );

	object_sub_ = n.subscribe("look_for_this", 1, &Tracker::stringCallback, this);
	camera_sub_ = n.subscribe(argv[2], 1, &Tracker::cameraCallback, this);
	image_sub_ = it_.subscribe(argv[1], 1, &Tracker::imageCallback, this);
	bound_pub_ = n_.advertise<geometry_msgs::Polygon>("/look_for_this/found_it",1);
}

~Tracker()
{
	cvDestroyWindow("Demo");
	cvDestroyWindow("Extra");
	cvReleaseImage(&image);
	cvReleaseImage(&frame);
	cvReleaseImage(&mask);
	cvReleaseImage(&backproject);
	cvReleaseImage(&sum);
	cvReleaseImage(&r);
	cvReleaseImage(&g);
	cvReleaseImage(&b);
}

void myBackProjectAndIntegrate( IplImage* img, IplImage* backprojection, IplImage* sum, const CvHistogram* hist )
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
	unsigned char* imgdata;
	double* dstdata;
	char *bpdata;
	double linetotal;
	
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
	
	imgdata= (uchar *)(img->imageData);
	dstdata= (double *)(sum->imageData);
	bpdata= (char *)(backprojection->imageData);
		

	int w = img->width;
	int h = img->height;
	int nc = img->nChannels;
	int step = img->widthStep/(nc*sizeof(uchar));
	for (int i=0; i<h; i++) {
		linetotal = 0.0;
		for (int j=0; j<w; j++) {
			linetotal+=(double)cvQueryHistValue_3D(hist,int( (imgdata[nc*j+0])/32),int( (imgdata[nc*j+1])/32),int( (imgdata[nc*j+2])/32));
			if( i<1 )
				(dstdata)[j]=linetotal;
			else
				(dstdata)[j]=(double)(dstdata)[j-step]+linetotal;
			(bpdata)[j]=(char)(cvQueryHistValue_3D(hist,int( (imgdata[nc*j+0])/32),int( (imgdata[nc*j+1])/32),int( (imgdata[nc*j+2])/32))>0)*255;
		}
		imgdata+=step*nc;
		dstdata+=step;
		bpdata+=step;
	}

}

void cameraCallback(const sensor_msgs::CameraInfoConstPtr& msg){
	for (int i=0; i<3; i++){
		for (int j=0; j<3; j++){
			K (i,j) = msg->K[i*3+j];
	}}
}

ublas::matrix<double> invertMatrix( ublas::matrix<double> ){
	ublas::matrix<double> invK (3, 3);
	double detK =  ( K(0,0)*K(1,1)*K(2,2) + K(0,1)*K(1,2)*K(2,0) + K(0,2)*K(1,0)*K(2,1) ) -
		( K(0,0)*K(1,2)*K(2,1) + K(0,1)*K(1,0)*K(2,2) + K(0,2)*K(1,1)*K(2,0) ); 
	invK(0,0) = ( K(1,1)*K(2,2) - K(1,2)*K(2,1) ) / detK;
	invK(0,1) = ( K(0,2)*K(2,1) - K(0,1)*K(2,2) ) / detK;
	invK(0,2) = ( K(0,1)*K(1,2) - K(0,2)*K(1,1) ) / detK;
	invK(1,0) = ( K(1,2)*K(2,0) - K(1,0)*K(2,2) ) / detK;
	invK(1,1) = ( K(0,0)*K(2,2) - K(0,2)*K(2,0) ) / detK;
	invK(1,2) = ( K(0,2)*K(1,0) - K(0,0)*K(1,2) ) / detK;
	invK(2,0) = ( K(1,0)*K(2,1) - K(1,1)*K(2,0) ) / detK;
	invK(2,1) = ( K(0,1)*K(2,0) - K(0,0)*K(2,1) ) / detK;
	invK(2,2) = ( K(0,0)*K(1,1) - K(0,1)*K(1,0) ) / detK;	

	return invK;
}

void stringCallback(const std_msgs::StringConstPtr& msg)
{
	object = strdup(msg->data.c_str());
	track_object = -1;

	histogram = fopen(object,"rb");
	fread(cvGetHistValue_3D(fg_hist,0,0,0),sizeof(float),dims[0]*dims[1]*dims[2],histogram);
	fclose(histogram);
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
		int scale_factor = 2;
		image = cvCreateImage( cvSize(cvGetSize(frame).width/scale_factor,cvGetSize(frame).height/scale_factor), 8, 3 );
		image->origin = frame->origin;
		mask = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 1 );
		sum = cvCreateImage( cvGetSize(image), IPL_DEPTH_64F, 1 );
		backproject = cvCreateImage( cvGetSize(image), IPL_DEPTH_8U, 1 );
		bg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
		lglikrat = cvCreateHist( 3, dims, CV_HIST_ARRAY, ranges, 1 );
		r = cvCreateImage( cvGetSize(image), 8, 1 );
		g = cvCreateImage( cvGetSize(image), 8, 1 );
		b = cvCreateImage( cvGetSize(image), 8, 1 );
	}

//	cvCopy( frame, image, 0 );
	cvResize( frame, image );

	if( track_object )
	{
		if( track_object < 0 )
		{
		
			cvSplit( image, b, g, r, 0 );
			planes[0]=b; planes[1] = g; planes[2]=r;
			
			float max_val = 0.f, min_val = 0.f;
			cvCalcHist( planes, bg_hist, 0, 0 );
//			cvGetMinMaxHistValue( fg_hist, 0, &max_val, 0, 0 );

	                cvGetMinMaxHistValue( fg_hist, &min_val, &max_val, 0, 0 );
	                cvConvertScale( fg_hist->bins, fg_hist->bins, max_val ? 255. / max_val : 0., 0 );
	                cvCalcBackProject( planes, backproject, fg_hist);
			cvThreshold( backproject, backproject, 127, 255, CV_THRESH_BINARY_INV);
       	       		cvShowImage( "Extra", backproject);
       			cvCalcHist( planes, bg_hist, 0, backproject );
       			cvCalcBackProject( planes, backproject, bg_hist);

			cvNormalizeHist(bg_hist, 1);
			cvNormalizeHist(fg_hist, 1);
			for( int m=0; m<8; m++){
			for( int n=0; n<8; n++){
			for( int o=0; o<8; o++){
				*(cvGetHistValue_3D(lglikrat,m,n,o)) = log((cvQueryHistValue_3D(fg_hist,m,n,o)+epsilon)/(cvQueryHistValue_3D(bg_hist,m,n,o)+epsilon));
			}}}
/**/			
			track_object = 1;
			
			
		}

		myBackProjectAndIntegrate( image, backproject, sum, lglikrat );
		
		h = sum->height;
		w = sum->width;
		step = sum->widthStep/sizeof(double);
		maxlik = -INFINITY;
		CvRect searchbox = cvRect(0, 0, 0, 0);
		CvRect foundbox = cvRect(0, 0, 0, 0);
		
		for (int i=cvGetSize(image).height; i>10; i=int(i/2)){
			searchbox = cvRect(0, 0, i, i);
			boxhstep = searchbox.height*step;
			sumdata= (double *)(sum->imageData);
			for (int j=0; j<(h-searchbox.height); j+=int(searchbox.height/8)){
				for (int k=0; k<(w-searchbox.width); k+=int(searchbox.width/8)){
					value = sumdata[k]+sumdata[k+searchbox.width+boxhstep]-sumdata[k+boxhstep]-sumdata[k+searchbox.width];
					if (value > maxlik){
						maxlik = value;
						foundbox = cvRect(k,j,searchbox.width, searchbox.height);
					}
				}
				sumdata+=step*int(searchbox.height/8);
			}
		}
/**/
//		cvRectangle(image,cvPoint(box.x,box.y),cvPoint(box.x+box.width,box.y+box.height),cvScalar(0,0,255),3,8,0);
		cvRectangle(image,cvPoint(foundbox.x,foundbox.y),cvPoint(foundbox.x+foundbox.width,foundbox.y+foundbox.height),cvScalarAll(255),1,8,0);	
		
//		cvShowImage( "Extra", backproject);
		
		cvSetImageROI( backproject, foundbox );
		cvSetImageROI( image, foundbox);
		contour = 0;
		cvFindContours( backproject, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		for( ; contour != 0; contour = contour->h_next )
		{
			CvScalar color = CV_RGB( rand()&255, rand()&255, rand()&255 );
			cvDrawContours( image, contour, color, color, -1, 1, 8 );
	        }
	        cvResetImageROI( backproject );
	        cvResetImageROI( image );

        	ublas::matrix<double> output (3,2);
        	printf("%i %i\n", foundbox.x, foundbox.y);
		output(0,0) = 2*foundbox.x;
		output(1,0) = 2*foundbox.y;
		output(2,0) = 2;
		output(0,1) = foundbox.x+foundbox.width;
		output(1,1) = foundbox.y+foundbox.height;
		output(2,1) = 2;

		output = ublas::prod (invertMatrix(K),output);

		foundpoints.set_points_size(1);
		foundpoints.points[0].x = output(0,0);
		foundpoints.points[0].y = output(1,0);
		foundpoints.points[0].z = output(2,0);
	        
/**/	}

	cvShowImage( "Demo", image );
//	cvShowImage( "Extra", backproject);
	cvWaitKey(3);

	try
	{
    	bound_pub_.publish(foundpoints);
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
ros::Subscriber object_sub_;
ros::Subscriber camera_sub_;
char** args;
bool show_image;
geometry_msgs::Polygon foundpoints;

IplImage *image, *frame, *mask, *backproject, *bp8U, *sum, *r, *g, *b, *planes[3];
CvHistogram *bg_hist, *fg_hist, *lglikrat;
CvMemStorage* storage;
CvSeq* contour;

int track_object, dims[3], thepoint[2], h, w, step, boxhstep;
float range_arr[2], *ranges[3], value, maxlik, epsilon;
double *sumdata;
char* object;
FILE *histogram;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tracker");
	ros::NodeHandle n;
	Tracker ic(n,argc,argv);
	ros::spin();
	return 0;
}


