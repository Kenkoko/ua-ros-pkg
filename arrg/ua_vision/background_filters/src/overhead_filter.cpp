#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "cv_bridge/CvBridge.h"
#include "background_filters/ObjectList.h"

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

using namespace std;

IplImage* bg = NULL; 
IplImage *original, *curr_image, *mono_image, *roi_color_image, *roi_bw_image;
CvMemStorage *storage;
CvSeq* contour;

bool isFirst = true;

sensor_msgs::CvBridge bridge;

ros::Publisher sub_pub, mask_pub, object_pub;

static void draw_box(IplImage *image, CvBox2D box, double color) {
  CvPoint2D32f boxPoints[4];

  cvBoxPoints(box, boxPoints);
  cvLineAA(image,
	   cvPoint((int)boxPoints[0].x, (int)boxPoints[0].y),
	   cvPoint((int)boxPoints[1].x, (int)boxPoints[1].y),
	   color);
  cvLineAA(image,
	   cvPoint((int)boxPoints[1].x, (int)boxPoints[1].y),
	   cvPoint((int)boxPoints[2].x, (int)boxPoints[2].y),
	   color);
  cvLineAA(image,
	   cvPoint((int)boxPoints[2].x, (int)boxPoints[2].y),
	   cvPoint((int)boxPoints[3].x, (int)boxPoints[3].y),
	   color);
  cvLineAA(image,
	   cvPoint((int)boxPoints[3].x, (int)boxPoints[3].y),
	   cvPoint((int)boxPoints[0].x, (int)boxPoints[0].y),
	   color);
}

void handleImage(const sensor_msgs::ImageConstPtr& msg_ptr) 
{
    try
	{
	    original = bridge.imgMsgToCv(msg_ptr, "rgb8");
	    
	    if (isFirst) {
		    curr_image = cvCreateImage(cvGetSize(original), 8, 3);
		    mono_image = cvCreateImage(cvGetSize(original), 8, 1);
            storage  = cvCreateMemStorage(0);
            contour = 0;
		    isFirst = false;
	    }
		
		cvSub(bg, original, curr_image);
		cvCvtColor(curr_image, mono_image, CV_RGB2GRAY);
		cvThreshold(mono_image, mono_image, 50, 255, CV_THRESH_BINARY);
		cvCopy(original, curr_image, mono_image);
		cvFindContours(mono_image, storage, &contour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		
		background_filters::ObjectList objectList;
		
		//cout << "HERE" << endl << endl;
		for( ; contour != 0; contour = contour->h_next )
		{
			CvScalar color = CV_RGB( rand()&255, rand()&255, rand()&255 );
			double area = fabs(cvContourArea(contour));
			//cout << area << endl;
			if (area > 200) 
			{
			    CvRect boundingBox = cvBoundingRect(contour);

                // Copy part of the original image that contains the contour
		        cvSetImageROI(original, boundingBox);
		        roi_color_image = cvCreateImage(cvSize(boundingBox.width, boundingBox.height), original->depth, original->nChannels);
		        cvCopy(original, roi_color_image);
		        cvResetImageROI(original);
		        objectList.color_images.push_back(*bridge.cvToImgMsg(roi_color_image, "rgb8"));
                cvReleaseImage(&roi_color_image);
                
                // Copy part of the black and white image that contains the contour
		        cvSetImageROI(mono_image, boundingBox);
		        roi_bw_image = cvCreateImage(cvSize(boundingBox.width, boundingBox.height), mono_image->depth, mono_image->nChannels);
		        cvCopy(mono_image, roi_bw_image);
		        cvResetImageROI(mono_image);
		        objectList.bw_images.push_back(*bridge.cvToImgMsg(roi_bw_image, "mono8"));
                cvReleaseImage(&roi_bw_image);

			    cvDrawContours(original, contour, color, color, -1, 2, 8 );
			    
			    CvBox2D minBox = cvMinAreaRect2(contour, storage);
			    draw_box(original, minBox, 1.0);

			    CvPoint topLeft, bottomRight;
			    topLeft.x = boundingBox.x;
			    topLeft.y = boundingBox.y;
			    bottomRight.x = boundingBox.x + boundingBox.width;
			    bottomRight.y = boundingBox.y + boundingBox.height;
			    cvRectangle(original, topLeft, bottomRight, color);
			    
			    sensor_msgs::RegionOfInterest roi;
			    roi.x_offset = boundingBox.x;
			    roi.y_offset = boundingBox.y;
			    roi.width = boundingBox.width;
			    roi.height = boundingBox.height;
			    
			    objectList.object_regions.push_back(roi);
		    }
        }	
	
		//objectList.originalImage = *bridge.cvToImgMsg(original, "rgb8");
		
		object_pub.publish(objectList);
		mask_pub.publish(bridge.cvToImgMsg(mono_image, "mono8"));
		sub_pub.publish(bridge.cvToImgMsg(original, "rgb8"));
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

    sub_pub = n.advertise<sensor_msgs::Image>("sub_image", 20);
    mask_pub = n.advertise<sensor_msgs::Image>("mask_image", 20);
    object_pub = n.advertise<background_filters::ObjectList>("object_list", 20);

    ros::Subscriber image_sub = n.subscribe("/camera/image_color", 20, handleImage);
    
    ros::spin();
    
    return 0;
}
