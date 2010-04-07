#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

using namespace std;

IplImage* bg = NULL; 
IplImage *original, *curr_image, *mono_image;

bool isFirst = true;

sensor_msgs::CvBridge bridge;

ros::Publisher sub_pub, mask_pub;

void handleImage(const sensor_msgs::ImageConstPtr& msg_ptr) 
{
    try
	{
	    original = bridge.imgMsgToCv(msg_ptr, "rgb8");
	    
	    if (isFirst) {
		    curr_image = cvCreateImage(cvGetSize(original), 8, 3);
		    mono_image = cvCreateImage(cvGetSize(original), 8, 1);
		    isFirst = false;
	    }
		
		cvSub(bg, original, curr_image);
		
		cvCvtColor(curr_image, mono_image, CV_RGB2GRAY);
		
		cvThreshold(mono_image, mono_image, 50, 255, CV_THRESH_BINARY);
				
		cvCopy(original, curr_image, mono_image);
		
		mask_pub.publish(bridge.cvToImgMsg(mono_image, "mono8"));
		sub_pub.publish(bridge.cvToImgMsg(curr_image, "rgb8"));
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error");
	}
}

int main (int argc, char **argv) 
{
    bg = cvLoadImage("/home/robotlab/Desktop/bg.jpg");
    
    ros::init(argc, argv, "overhead_filter_node");
    
    ros::NodeHandle n;

    sub_pub = n.advertise<sensor_msgs::Image>("/sub_image", 20);
    mask_pub = n.advertise<sensor_msgs::Image>("/mask_image", 20);

    ros::Subscriber image_sub = n.subscribe("/camera/image_color", 20, handleImage);
    
    ros::spin();
    
    return 0;
}
