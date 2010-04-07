/*********************************************

try:

	rosrun color_based_tracking camshift5 true overhead_cam/image_raw histograms/white_block_hist

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


IplImage *image, *frame, *mask, *backproject, *bp8U, *sum, *r, *g, *b, *planes[3];
CvHistogram *bg_hist, *fg_hist, *lglikrat;
CvMemStorage* storage;
CvSeq* contour;

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
float range_arr[2], *ranges[3], value, maxlik, epsilon;
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
	storage = cvCreateMemStorage(0);
	contour = 0;

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
	cvReleaseImage(&image);
	cvReleaseImage(&frame);
	cvReleaseImage(&mask);
	cvReleaseImage(&backproject);
	cvReleaseImage(&sum);
	cvReleaseImage(&r);
	cvReleaseImage(&g);
	cvReleaseImage(&b);
}

void myBackProjectAndIntegrate( IplImage** img, IplImage* backprojection, IplImage* sum, const CvHistogram* hist )
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
	unsigned char* imgdata[3];
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
	
	for( int i=0; i<3; i++)
		imgdata[i]= (uchar *)(img[i]->imageData);
	dstdata= (double *)(sum->imageData);
	bpdata= (char *)(backprojection->imageData);
		

	int w = img[0]->width;
	int h = img[0]->height;
	int step = img[0]->widthStep/sizeof(uchar);
//	int nc = img[0]->nChannels;
	for (int i=0; i<h; i++) {
		linetotal = 0.0;
		for (int j=0; j<w; j++) {
			linetotal+=(double)cvQueryHistValue_3D(hist,int( (imgdata[0][j])/32),int( (imgdata[1][j])/32),int( (imgdata[2][j])/32));
			if( i<1 )
				(dstdata)[j]=linetotal;
			else
				(dstdata)[j]=(double)(dstdata)[j-step]+linetotal;
/**/			(bpdata)[j]=(char)(cvQueryHistValue_3D(hist,int( (imgdata[0][j])/32),int( (imgdata[1][j])/32),int( (imgdata[2][j])/32))>0)*255;
		}
		imgdata[0]+=step;
		imgdata[1]+=step;
		imgdata[2]+=step;
		dstdata+=step;
		bpdata+=step;
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	box = selection;
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
		image = cvCreateImage( cvSize(cvGetSize(frame).width/2,cvGetSize(frame).height/2), 8, 3 );
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
		cvSplit( image, r, g, b, 0 );
		planes = {r,g,b};

		if( track_object < 0 )
		{
			cvZero(mask);
			float max_val = 0.f;
			if( !object ){
				cvRectangle(mask,cvPoint(selection.x,selection.y),cvPoint(selection.x+selection.width,selection.y+selection.height),cvScalarAll(255),CV_FILLED,8,0);
				cvCalcHist( planes, fg_hist, 0, mask);
			}
			cvNot(mask,mask);
			cvCalcHist( planes, bg_hist, 0, mask );
			cvNot(mask,mask);
			cvGetMinMaxHistValue( fg_hist, 0, &max_val, 0, 0 );
			cvCalcHist( planes, bg_hist, 0, 0 );
			cvNormalizeHist(bg_hist, max_val);
			for( int m=0; m<8; m++){
			for( int n=0; n<8; n++){
			for( int o=0; o<8; o++){
				*(cvGetHistValue_3D(lglikrat,m,n,o)) = log((cvQueryHistValue_3D(fg_hist,m,n,o)+epsilon)/(cvQueryHistValue_3D(bg_hist,m,n,o)+epsilon));
			}}}
			track_window = selection;
			track_object = 1;
		}

//		cvCalcBackProject( planes, backproject, lglikrat );
//		cvIntegral(backproject, sum, 0, 0);
		myBackProjectAndIntegrate( planes, backproject, sum, lglikrat );
//		cvThreshold(backproject, bp8U, 0, 255, CV_THRESH_BINARY);
		
		h = sum->height;
		w = sum->width;
		step = sum->widthStep/sizeof(double);
		maxlik = -INFINITY;
		CvRect searchbox = cvRect(0, 0, 0, 0);
		CvRect thebox = cvRect(0, 0, 0, 0);
		
		for (int i=cvGetSize(image).height; i>10; i=int(i/2)){
			searchbox = cvRect(0, 0, i, i);
			boxhstep = searchbox.height*step;
			sumdata= (double *)(sum->imageData);
			for (int j=0; j<(h-searchbox.height); j+=int(searchbox.height/8)){
				for (int k=0; k<(w-searchbox.width); k+=int(searchbox.width/8)){
					value = sumdata[k]+sumdata[k+searchbox.width+boxhstep]-sumdata[k+boxhstep]-sumdata[k+searchbox.width];
					if (value > maxlik){
						maxlik = value;
						thebox = cvRect(k,j,searchbox.width, searchbox.height);
					}
				}
				sumdata+=step*int(searchbox.height/8);
			}
		}
/**/
		cvRectangle(image,cvPoint(box.x,box.y),cvPoint(box.x+box.width,box.y+box.height),cvScalar(0,0,255),3,8,0);
		cvRectangle(image,cvPoint(thebox.x,thebox.y),cvPoint(thebox.x+thebox.width,thebox.y+thebox.height),cvScalarAll(255),1,8,0);	
		
		cvShowImage( "Extra", backproject);
		
		cvSetImageROI( backproject, thebox );
		cvSetImageROI( image, thebox);
		contour = 0;
		cvFindContours( backproject, storage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		for( ; contour != 0; contour = contour->h_next )
		{
			CvScalar color = CV_RGB( rand()&255, rand()&255, rand()&255 );
			cvDrawContours( image, contour, color, color, -1, 2, 8 );
	        }
	        cvResetImageROI( backproject );
	        cvResetImageROI( image );
/**/	}

	if( select_object && selection.width > 0 && selection.height > 0 )
	{
		cvSetImageROI( image, selection );
		cvXorS( image, cvScalarAll(255), image, 0 );
		cvResetImageROI( image );
	}
/**/

	cvShowImage( "Demo", image );
//	cvShowImage( "Extra", backproject);
	cvWaitKey(3);

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
	Tracker ic(n,argc,argv);
	ros::spin();
	return 0;
}


