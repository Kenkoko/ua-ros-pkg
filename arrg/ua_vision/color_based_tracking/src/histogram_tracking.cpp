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

//#define lglikrat( j ) log(((fg_data[0])[j]*(fg_data[0])[j+1]*(fg_data[0])[j+2]+eps)/((bg_data[0])[j]*(bg_data[0])[j+1]*(bg_data[0])[j+2]+eps))
#define lglikrat( j ) log((fg_data[j]*fg_data[j+1]*fg_data[j+2]+eps)/(bg_data[j]*bg_data[j+1]*bg_data[j+2]+eps))
//#define lglikrat( j ) log((fg_data[0][j]*fg_data[1][j]*fg_data[2][j]+eps)/(bg_data[0][j]*bg_data[1][j]*bg_data[2][j]+eps))
//#define lglikrat( j ) abs(log((fg_data[j]+eps)/(bg_data[j]+eps)))

class ImageConverter {

public:

ImageConverter(ros::NodeHandle &n, char** argv) :
	n_(n), it_(n_)
{
	args = argv;
	show_image = (bool) (((std::string) args[1])=="true");
/*
	bound_pub_ = n_.advertise<geometry_msgs::Polygon>("/color_tracking/boundbox_"+red+"_"+green+"_"+blue,1);
	if( show_image )
		image_pub_ = it_.advertise("/color_tracking/image_"+red+"_"+green+"_"+blue,1);
/**/
	cvNamedWindow("Image window");
	cvNamedWindow("B");
	cvNamedWindow("F");
/*	cvNamedWindow("r");
	cvNamedWindow("g");
	cvNamedWindow("b");
*/	image_sub_ = it_.subscribe(args[2], 1, &ImageConverter::imageCallback, this);

}

~ImageConverter()
{
	cvDestroyWindow("Image window");
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


/*
	IplImage* h_plane = cvCreateImage( cvGetSize(cv_image), 8, 1 );
        IplImage* s_plane = cvCreateImage( cvGetSize(cv_image), 8, 1 );
        IplImage* v_plane = cvCreateImage( cvGetSize(cv_image), 8, 1 );
        IplImage* planes[] = { h_plane, s_plane };
        IplImage* hsv = cvCreateImage( cvGetSize(cv_image), 8, 3 );
        int h_bins = 30, s_bins = 32;
        int hist_size[] = {h_bins, s_bins};
        float h_ranges[] = { 0, 180 };
        float s_ranges[] = { 0, 255 };
        float* ranges[] = { h_ranges, s_ranges };
        int scale = 10;
        IplImage* hist_img =
            cvCreateImage( cvSize(h_bins*scale,s_bins*scale), 8, 3 );
        CvHistogram* hist;
        float max_value = 0;
        int h, s;

        cvCvtColor( cv_image, hsv, CV_BGR2HSV );
        cvCvtPixToPlane( hsv, h_plane, s_plane, v_plane, 0 );
        hist = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
        cvCalcHist( planes, hist, 0, 0 );
        cvGetMinMaxHistValue( hist, 0, &max_value, 0, 0 );
        cvZero( hist_img );

        for( h = 0; h < h_bins; h++ )
        {
            for( s = 0; s < s_bins; s++ )
            {
                float bin_val = cvQueryHistValue_2D( hist, h, s );
                int intensity = cvRound(bin_val*255/max_value);
                cvRectangle( hist_img, cvPoint( h*scale, s*scale ),
                             cvPoint( (h+1)*scale - 1, (s+1)*scale - 1),
                             CV_RGB(intensity,intensity,intensity),
                             CV_FILLED );
            }
        }

        cvNamedWindow( "Source", 1 );
        cvShowImage( "Source", cv_image );

        cvNamedWindow( "H-S Histogram", 1 );
        cvShowImage( "H-S Histogram", hist_img );

*/

	// set the target colour from the command line params
//	CvScalar targetColour = CV_RGB((uchar)atoi(args[3]), (uchar)atoi(args[4]), (uchar)atoi(args[5]));

// { Begin hist tracking code

	dims = {8,8,8};
	range = { 0,255 };
	hranges = {range,range,range};
	ranges = {range};

//	box = cvRect(580,165,620-580,220-165); //grey box
	box = cvRect(788,106,855-788,177-106); //gold box
//	box = cvRect(393,138,440-393,201-138); //white box
	mask = cvCreateImage(cvGetSize(cv_image),8,1);
	cvZero(mask);
	cvRectangle(mask,cvPoint(box.x,box.y),cvPoint(box.x+box.width,box.y+box.height),cvScalarAll(255),CV_FILLED,8,0);

	r = cvCreateImage( cvGetSize(cv_image), 8, 1 );
        g = cvCreateImage( cvGetSize(cv_image), 8, 1 );
        b = cvCreateImage( cvGetSize(cv_image), 8, 1 );
        cvSplit(cv_image, r, g, b, NULL);
/*	cvShowImage( "r", r );
	cvShowImage( "g", g );
	cvShowImage( "b", b ); */
        planes = { r,g,b };

		bg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, hranges, 1);
		fg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, hranges, 1);

		cvCalcHist(planes, bg_hist, 0, mask);
		cvNormalizeHist(bg_hist, 1);
		cvNot(mask,mask);
		cvCalcHist(planes, fg_hist, 0, mask);
		cvNormalizeHist(fg_hist, 1);

		bg_histimg = cvCreateImage( cvGetSize(cv_image), IPL_DEPTH_8U, 1 );
		cvCalcArrBackProject( (void**)planes, bg_histimg, bg_hist);
		fg_histimg = cvCreateImage( cvGetSize(cv_image), IPL_DEPTH_8U, 1 );
		cvCalcArrBackProject( (void**)planes, fg_histimg, fg_hist);

		bg_data= reinterpret_cast<unsigned char *>(bg_histimg->imageData);
		fg_data= reinterpret_cast<unsigned char *>(fg_histimg->imageData);


//	cvShowImage( "F", fg_histimg );
//	cvShowImage( "B", bg_histimg );

/*
	for( int i=0; i<3; i++) {
		bg_hist[i] = cvCreateHist( 1, &(dims[0]), CV_HIST_ARRAY, ranges, 1);
		fg_hist[i] = cvCreateHist( 1, &(dims[0]), CV_HIST_ARRAY, ranges, 1);

		cvCalcHist(&(planes[i]), bg_hist[i], 0, mask);
		cvNot(mask,mask);
		cvCalcHist(&(planes[i]), fg_hist[i], 0, mask);

		bg_histimg[i] = cvCreateImage( cvGetSize(cv_image), 8, 1 );
		cvCalcBackProject(&(planes[i]), bg_histimg[i], bg_hist[i]);
		fg_histimg[i] = cvCreateImage( cvGetSize(cv_image), 8, 1 );
		cvCalcBackProject(&(planes[i]), fg_histimg[i], fg_hist[i]);

		bg_data[i]= reinterpret_cast<unsigned char *>(bg_histimg[i]->imageData);
		fg_data[i]= reinterpret_cast<unsigned char *>(fg_histimg[i]->imageData);
	}
	cvShowImage( "extra", bg_histimg[0] );
*/

/*
	printf("\nHist:");
	for( int m=0; m<8; m++){
	printf("\n");
	for( int n=0; n<8; n++){
	printf("\n");
	for( int o=0; o<8; o++){
		printf("%4.4g ",cvQueryHistValue_3D(fg_hist,m,n,o));
	}}}
*/


	eps = 0.000000001;
	w = bg_histimg->width;//cv_image->width;
	h = bg_histimg->height;//cv_image->height;
	step = bg_histimg->widthStep;//cv_image->widthStep;
	nc = bg_histimg->nChannels;//= cv_image->nChannels;
	likrat = cvCreateMat(cv_image->width, cv_image->height,CV_32FC1); 



	for (int i=box.y; i<box.y+box.height; i++) {
		p=fg_data+i*step;
		for (int j=box.x; j<box.x+box.width; j++) {
			printf("%4.4g ", p[j]);
		}
		printf("\n");
	}


	cvmSet(likrat,0,0,lglikrat(0)); // lglikrat is a macro

	for (int j=1; j<w; j++) {
		cvmSet(likrat,0,j,cvmGet(likrat,0,j-1)+lglikrat(nc*j));
	}

	for (int i=1; i<h; i++) {
		bg_data+= step;  // next line
		fg_data+= step;

		line_total = 0.0;
		for (int j=0; j<w; j++) {
			line_total+= lglikrat(nc*j);
			cvmSet(likrat,i,j,cvmGet(likrat,i-1,j)+line_total);
		}
	}

	maxlik = 0.0;
	value = 0.0;
	thepoint = {0,0};	
	for (int i=0; i<h-box.height; i++) {
		for (int j=0; j<w-box.width; j++) {
			value = cvmGet(likrat,i,j)+cvmGet(likrat,i+box.height,j+box.width)-cvmGet(likrat,i+box.height,j)+cvmGet(likrat,i,j+box.width);
//			printf("%f, %f\n", value, maxlik);
			if (value > maxlik){
//				printf("%f, %f\n", value, maxlik);
				maxlik = value;
				thepoint = {j,i};
			}
		}
	}

	cvRectangle(cv_image,cvPoint(box.x,box.y),cvPoint(box.x+box.width,box.y+box.height),cvScalar(0,0,255),1,8,0);
	cvRectangle(cv_image,cvPoint(thepoint[0],thepoint[1]),cvPoint(thepoint[0]+box.width,thepoint[1]+box.height),cvScalarAll(255),1,8,0);	
/**/

	for ( int i=0; i<3; i++){
		cvReleaseImage(&planes[i]);
	}

	if( show_image ){
		cvShowImage("Image window", cv_image);
		cvWaitKey(3);
	}

	try
	{
//		bound_pub_.publish(boxes);
//		if( show_image ){
//			image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8")); }
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error");
	}

//	cvReleaseImage(&cv_image);
	cvReleaseImage(&mask);
	cvReleaseImage(&bg_histimg);
	cvReleaseImage(&fg_histimg);

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
//wubble_vision::bounding_box boxes;

IplImage *cv_image,*mask,*r,*g, *b,*planes[3];
IplImage *bg_histimg, *fg_histimg;
int dims[3], w, h, step, nc, thepoint[2];
float range[2], eps, line_total, maxlik, value, *hranges[3], *ranges[3];;
CvRect box;
CvHistogram *bg_hist, *fg_hist;
unsigned char *bg_data, *fg_data, *bg_dataroi, *fg_dataroi;
CvMat* likrat;
unsigned char* p;


};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle n;
	ImageConverter ic(n,argv);
	ros::spin();
	return 0;
}


