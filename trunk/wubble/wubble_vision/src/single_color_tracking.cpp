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

class ImageConverter {

public:

ImageConverter(ros::NodeHandle &n, char** argv) :
	n_(n), it_(n_)
{
	args = argv;
	bool show_image = ((std::string) args[1])=="true";
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


// { Begin color tracking code

	const int tolerance = 8; // how far away from the colour can be accepted the same colour?

	// set the target colour from the command line params
	CvScalar targetColour = CV_RGB((uchar)atoi(args[3]), (uchar)atoi(args[4]), (uchar)atoi(args[5]));

	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* comp = NULL;
	IplImage* dst = cvCreateImage( cvGetSize(cv_image), cv_image->depth, cv_image->nChannels);
	cvPyrSegmentation( cv_image, dst, storage, &comp, 10, 200, 50 );
	int n_comp = comp->total;

	for( int i=0; i<n_comp; i++ ) {
		CvConnectedComp* cc = (CvConnectedComp*) cvGetSeqElem( comp, i );
		CvRect rect = cc->rect;
		cvSetImageROI(cv_image, rect);
		CvPoint seeds[5] = { cvPoint(rect.width/10,rect.height/10), cvPoint(rect.width/10, int(rect.height*.9)), cvPoint(int(rect.width*.9),rect.height/10), cvPoint(int(rect.width*.9), int(rect.height*.9)), cvPoint(rect.width/2, rect.height/2)};
		for( int j=0; j<5; j++) {
			printf("<%d,%d>\n",seeds[j].x,seeds[j].y);
			cvFloodFill(cv_image, seeds[j], CV_RGB(0,0,0), cvScalarAll(5.0), cvScalarAll(5.0), cc, 4+CV_FLOODFILL_MASK_ONLY, cvCreateImage( cvSize(cvGetSize(cv_image).width+2,cvGetSize(cv_image).height+2), IPL_DEPTH_8U,1));
			if (cc->value.val[0] >= targetColour.val[0]-tolerance && // compare BGR
				cc->value.val[0] <= targetColour.val[0]+tolerance &&
				cc->value.val[1] >= targetColour.val[1]-tolerance &&
				cc->value.val[1] <= targetColour.val[1]+tolerance &&
				cc->value.val[2] >= targetColour.val[2]-tolerance &&
				cc->value.val[3] <= targetColour.val[2]+tolerance &&
				cc->area >= 10) {

				rect = cc->rect;
				cvRectangle(cv_image, cvPoint(rect.x,rect.y), cvPoint(rect.x+rect.width,rect.y+rect.height), CV_RGB(255,0,0), 3, 0, 0);

//				this->boxes.set_boxes_size(5);
//				this->boxes[j].boxes.set_points_size(2)
//				this->boxes[j].boxes[0].x = rect.x;
//				this->boxes[j].boxes[0].y = rect.y;
//				this->boxes[j].boxes[1].x = rect.x+rect.width;
//				this->boxes[j].boxes[1].y = rect.y+rect.height;
			}
		}
		cvResetImageROI(cv_image);
	}
	cvReleaseMemStorage( &storage );



/**	
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
	this->boundbox.set_points_size(2);
	this->boundbox.points[0].x = 0;
	this->boundbox.points[0].y = 0;
	this->boundbox.points[1].x = 0;
	this->boundbox.points[1].y = 0;
**/

// } End color tracking



	if( show_image ){
		//cvRectangle(cv_image, cvPoint(minx,miny), cvPoint(maxx,maxy), CV_RGB(255,0,0), 3, 0, 0);
		cvShowImage("Image window", cv_image);
		cvWaitKey(3);
	}

	try
	{
		bound_pub_.publish(boxes);
//		if( show_image == "true" )
//			image_pub_.publish(bridge_.cvToImgMsg(cv_image, "bgr8"));
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
wubble_vision::bounding_box boxes;


};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle n;
	ImageConverter ic(n,argv);
	ros::spin();
	return 0;
}


