#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <boost/thread/thread.hpp>

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

using namespace std;

#define NUM_BGS 20

IplImage *bgs[NUM_BGS];
IplImage *ave_bg;
int bg_counter = 0;
bool have_ave_bg = false;

sensor_msgs::Image::Ptr ave_msg;
ros::Subscriber image_sub;
ros::Publisher ave_bg_pub;

void publish_average_background()
{
    sensor_msgs::CvBridge b;
    ros::Rate r(50.0);

    while (ros::ok())
    {
        if (have_ave_bg)
        {
            ave_bg_pub.publish(b.cvToImgMsg(ave_bg, "bgr8"));
            r.sleep();
        }
    }
}

void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
{
	if (bg_counter < NUM_BGS)
	{
        sensor_msgs::CvBridge bridge;
        IplImage *bg = NULL;

        try
	    {
	        bg = bridge.imgMsgToCv(msg_ptr, "bgr8");
	        bgs[bg_counter++] = cvCloneImage(bg);
	        //cout << "bgs[" << bg_counter << "] = " << bgs[bg_counter] << endl;
	        //ros::Duration(1.0).sleep();
        }
	    catch (sensor_msgs::CvBridgeException error)
	    {
		    ROS_ERROR("CvBridgeError");
	    }
	    
	    return;
	}
	
	if (!have_ave_bg)
	{
        sensor_msgs::CvBridge bridge;
	    
        int img_width = bgs[0]->width;
        int img_height = bgs[0]->height;
        int n_channels = bgs[0]->nChannels;
	    cout << img_width << endl;
	    cout << img_height << endl;
	    cout << n_channels << endl;
	    
	    ave_bg = cvCreateImage(cvSize(img_width, img_height), bgs[0]->depth, n_channels);
	    uchar *ave_data = (uchar *) ave_bg->imageData;
	    
        for (int i = 0; i < bgs[0]->imageSize; ++i)
        {
            double sum = 0.0;
            
	        for (int j = 0; j < NUM_BGS; ++j)
            {
                sum += bgs[j]->imageData[i];
            }
            
            ave_data[i] = (int) (sum / NUM_BGS);
        }
        
        for (int j = 0; j < NUM_BGS; ++j)
        {
            cvReleaseImage(&bgs[j]);
        }
        
        have_ave_bg = true;
	}
	else
	{
	    image_sub.shutdown();
	}
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "background_sampler");
    ros::NodeHandle n;
    
    image_sub = n.subscribe("/camera/image_color", 1, handle_image);
    ave_bg_pub = n.advertise<sensor_msgs::Image>("average_bg", 1);

    boost::thread t = boost::thread::thread(boost::bind(&publish_average_background));
    ros::spin();

    t.interrupt();
    t.join();
    
    return 0;
}
