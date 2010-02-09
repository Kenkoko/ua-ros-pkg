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

#define lglikrat( j ) log((fg_data[j]*fg_data[j+1]*fg_data[j+2]+eps)/(bg_data[j]*bg_data[j+1]*bg_data[j+2]+eps))

class ImageConverter {

public:

ImageConverter(ros::NodeHandle &n, char** argv) :
	n_(n), it_(n_)
{
	args = argv;
	show_image = (bool) (((std::string) args[1])=="true");
/*	std::string red = args[3];
	std::string green = args[4];
	std::string blue = args[5];
	bound_pub_ = n_.advertise<geometry_msgs::Polygon>("/color_tracking/boundbox_"+red+"_"+green+"_"+blue,1);
	if( show_image )
		image_pub_ = it_.advertise("/color_tracking/image_"+red+"_"+green+"_"+blue,1);
*/
	cvNamedWindow("Image window");
	image_sub_ = it_.subscribe(args[2], 1, &ImageConverter::imageCallback, this);

}

~ImageConverter()
{
	cvDestroyWindow("Image window");
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
{

	IplImage *cv_image = NULL;
	try
	{
		cv_image = bridge_.imgMsgToCv(msg_ptr, "bgr8");
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error");
	}

	// set the target colour from the command line params
	CvScalar targetColour = CV_RGB((uchar)atoi(args[3]), (uchar)atoi(args[4]), (uchar)atoi(args[5]));

// { Begin hist tracking code

	int dims[] = {8,8,8};
	float range[] = { 0,255 };
	float* hranges[] = {range,range,range};


//	CvHistogram* bg_hist = cvCreateHist( dims, sizes*, type, ranges, uniform); 
	CvHistogram* bg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, hranges, 1);
	CvHistogram* fg_hist = cvCreateHist( 3, dims, CV_HIST_ARRAY, hranges, 1);

//	grey box: 580,165 - 620,220
//	gold box: 788,106 - 855,177
//	white box: 393,138 - 440,201
	CvRect box = cvRect(580,165,620-580,220-165);
	IplImage* mask = cvCreateImage(cvGetSize(cv_image),8,1);
	cvRectangle(mask,cvPoint(580,165),cvPoint(620,220),cvScalarAll(255),CV_FILLED,8,0);
	
	cvCalcHist(&cv_image, bg_hist, 0, mask);
	cvNot(mask,mask);
	cvCalcHist(&cv_image, fg_hist, 0, mask);

	IplImage* bg_histimg = cvCloneImage(cv_image);
	cvCalcBackProject(cv_image, bg_histimg, bg_hist);
	IplImage* fg_histimg = cvCloneImage(cv_image);
	cvCalcBackProject(cv_image, fg_histimg, fg_hist);

	float eps = 0.000000001;
	float line_total;

	int nl = cv_image->height; // number of lines
	int nc = cv_image->width * cv_image->nChannels; // total number of element per line
	int step= cv_image->widthStep; // effective width

	unsigned char* bg_data= reinterpret_cast<unsigned char *>(bg_histimg->imageData);
	unsigned char* fg_data= reinterpret_cast<unsigned char *>(fg_histimg->imageData);
	CvMat* likrat = cvCreateMat(cv_image->width, cv_image->height,CV_32FC1); 

	// lglikrat is a macro
	cvmSet(likrat,0,0,lglikrat(0));

	for (int j=1; j<nc; j+= cv_image->nChannels) {

		cvmSet(likrat,0,j,cvmGet(likrat,0,j-1)+lglikrat(j));

	} // end of line

	for (int i=2; i<=nl; i++) {
		bg_data+= step;  // next line
		fg_data+= step;

		line_total = 0.0;
		for (int j=0; j<nc; j+= cv_image->nChannels) {
			line_total+= lglikrat(j);
			cvmSet(likrat,i-1,j,cvmGet(likrat,i-1,j)+line_total);
		}
	}

	float maxlik = 0.0;
	float value = 0.0;
	int thepoint[2] = {0,0};	
	for (int i=0; i<nl-box.height; i++) {
		for (int j=0; j<likrat->width-box.width; j++) {
			value = cvmGet(likrat,i,j)+cvmGet(likrat,i+box.height,j+box.width)-cvmGet(likrat,i+box.height,j)+cvmGet(likrat,i,j+box.width);
			if (value > maxlik){
				maxlik = value;
				thepoint = {i,j};
			}
		}
	}

	cvRectangle(mask,cvPoint(thepoint[0],thepoint[1]),cvPoint(thepoint[0]+box.height,thepoint[1]+box.width),cvScalarAll(255),1,8,0);	

	if( show_image ){
//		cvRectangle(cv_image, cvPoint(minx,miny), cvPoint(maxx,maxy), CV_RGB(255,0,0), 3, 0, 0);
//I		IplImage *dst = cvCreateImage( cvGetSize(cv_image), IPL_DEPTH_8U,1);
//		cvCvtColor(cv_image, dst, CV_RGB2GRAY);
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



};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle n;
	ImageConverter ic(n,argv);
	ros::spin();
	return 0;
}


