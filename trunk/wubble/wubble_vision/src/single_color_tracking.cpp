/*********************************************

Creates node for single color tracking.

	single_color_tracking /image_stream red green blue

For example:

	rosrun wubble_vision single_color_tracking /stereo/left/image_color 0 255 0

Should view through erratic's camera, and recognize green

Written by: Jeremy Wright
with code stolen from various sources

*********************************************/

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <stdlib.h>
#include <stdio.h>

class ImageConverter {

public:

ImageConverter(ros::NodeHandle &n, char** argv) :
  n_(n), it_(n_)
{
  args = argv;
  image_pub_ = it_.advertise("image_topic_2",1);

  cvNamedWindow("Image window");
  image_sub_ = it_.subscribe(
    args[1], 1, &ImageConverter::imageCallback, this);
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


// { Begin color tracking code

	const int tolerance = 8; // how far away from the colour can be accepted the same colour?

	// set the target colour from the command line params
	CvScalar targetColour = CV_RGB((uchar)atoi(args[2]), (uchar)atoi(args[3]), (uchar)atoi(args[4]));
	
	int w,h;
	CvScalar white = CV_RGB(255,255,255);
	CvScalar black = CV_RGB(0,0,0);
	IplImage *clone = cvCloneImage(cv_image);
	//fprintf(stdout, "image cloned, processing colour mask..\n");
	int minx=10000, miny=10000;
	int maxx=0, maxy=0;
	for (h=0; h<clone->height; h++) for (w=0; w<clone->width; w++) {
		CvScalar currentColour = cvGet2D(clone,h,w);
		if (currentColour.val[0] >= targetColour.val[0]-tolerance && // compare BGR
				currentColour.val[0] <= targetColour.val[0]+tolerance &&
			  currentColour.val[1] >= targetColour.val[1]-tolerance &&
				currentColour.val[1] <= targetColour.val[1]+tolerance &&
				currentColour.val[2] >= targetColour.val[2]-tolerance &&
				currentColour.val[3] <= targetColour.val[2]+tolerance ) {
			  	// if they are the same then make this cell white
			  	cvSet2D(clone,h,w,white);
			  	if (w>maxx) maxx=w; if (h>maxy) maxy=h;
			  	if (w<minx) minx=w; if (h<miny) miny=h;
			  	//fprintf(stdout, "colour match found at %dx%d\n", w, h);
		} else {
				// if they are NOT the same then make this cell black
			  	cvSet2D(clone,h,w,black);
		}
	}

	//fprintf(stdout, "mask complete, showing results in mask window\n");
	cvRectangle(cv_image, cvPoint(minx,miny), cvPoint(maxx,maxy), CV_RGB(255,0,0), 3, 0, 0);

// } End color tracking

  cvShowImage("Image window", cv_image);
  cvWaitKey(3);

  try
  {
    image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8"));
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
char** args;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;
  ImageConverter ic(n,argv);
  ros::spin();
  return 0;
}


