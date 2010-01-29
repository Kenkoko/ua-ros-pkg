/*********************************************

Creates node for single color tracking.

	single_color_tracking boolean-pub-image /image_stream red green blue

For example:

	rosrun wubble_vision single_color_tracking true /stereo/left/image_color 0 255 0

Should view through erratic's camera, and recognize green,
amakend output an image with that color outlined

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

#include "BlobResult.h"

class ImageConverter {

public:

ImageConverter(ros::NodeHandle &n, char** argv) :
	n_(n), it_(n_)
{
	args = argv;
	show_image = (bool) (((std::string) args[1])=="true");
	printf("%i\n", show_image);
	std::string red = args[3];
	std::string green = args[4];
	std::string blue = args[5];
	bound_pub_ = n_.advertise<geometry_msgs::Polygon>("/color_tracking/boundbox_"+red+"_"+green+"_"+blue,1);
	if( show_image )
		image_pub_ = it_.advertise("/color_tracking/image_"+red+"_"+green+"_"+blue,1);

	cvNamedWindow("Image window");
	image_sub_ = it_.subscribe(
		args[2], 1, &ImageConverter::imageCallback, this);
}

~ImageConverter()
{
	//cvDestroyWindow("Image window");
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


	//////////////////////////////////////////////////////////////
	// get blobs and filter them using its area
	/////////////////////////////////////////////////////////////
	CBlobResult blobs;
	int i;
	CBlob *currentBlob;
	IplImage *original, *originalThr;

	// load an image and threshold it
	original = cv_image;
	cvThreshold( original, originalThr, 100, 255, CV_THRESH_BINARY );
	printf("1");
	// find non-white blobs in thresholded image
	blobs = CBlobResult( originalThr, NULL, 255, false );
	// exclude the ones smaller than param2 value
	int param2 = 0;
	blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, param2 );
	printf("2");
	// get mean gray color of biggest blob
	CBlob biggestBlob;
//	CBlobGetMean getMeanColor( original );
	double meanGray;
	printf("3");
	blobs.GetNthBlob( CBlobGetArea(), 0, biggestBlob );
	meanGray = biggestBlob.Mean();//getMeanColor( biggestBlob );
	printf("4");
	// display filtered blobs
	IplImage *displayedImage = cvCloneImage(originalThr);
	cvMerge( originalThr, originalThr, originalThr, NULL, displayedImage );
	printf("5");
	for (i = 0; i < blobs.GetNumBlobs(); i++ )
	{
	        currentBlob = blobs.GetBlob(i);
	        currentBlob->FillBlob( displayedImage, CV_RGB(255,0,0));
	}
	printf("6");

	this->boxes.set_points_size(2);
	this->boxes.points[0].x = 0;
	this->boxes.points[0].y = 0;
	this->boxes.points[1].x = 0;
	this->boxes.points[1].y = 0;

	cv_image = original;


	if( show_image ){
//		cvRectangle(cv_image, cvPoint(minx,miny), cvPoint(maxx,maxy), CV_RGB(255,0,0), 3, 0, 0);
		cvShowImage("Image window", cv_image);
		cvWaitKey(3);
	}

	try
	{
		bound_pub_.publish(boxes);
		if( show_image ){
			image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8")); }
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
//wubble_vision::bounding_box boxes;\

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle n;
	ImageConverter ic(n,argv);
	ros::spin();
	return 0;
}


