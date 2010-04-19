#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <boost/thread/thread.hpp>
#include <math.h>
#include <background_filters/GetBgStats.h>

#include <sys/types.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

using namespace std;

#define NUM_BGS 100

IplImage *bgs[NUM_BGS];
IplImage *ave_bg;
vector<double> std_dev;
int bg_counter = 0;
bool have_ave_bg = false;

ros::ServiceServer service;
ros::Subscriber image_sub;
ros::Publisher ave_bg_pub;

void publish_average_background()
{
    sensor_msgs::CvBridge b;
    ros::Rate r(1.0);

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
        }
	    catch (sensor_msgs::CvBridgeException error)
	    {
		    ROS_ERROR("CvBridgeError");
	    }
	    
	    return;
	}
	
	if (!have_ave_bg)
	{
        int img_width = bgs[0]->width;
        int img_height = bgs[0]->height;
        int n_channels = bgs[0]->nChannels;
	    cout << img_width << endl;
	    cout << img_height << endl;
	    cout << n_channels << endl;
	    
	    cout << "depth = " << bgs[0]->depth << endl;
	    cout << IPL_DEPTH_8U << endl;
	    
	    ave_bg = cvCreateImage(cvSize(img_width, img_height), bgs[0]->depth, n_channels);
	    uchar *ave_data = (uchar *) ave_bg->imageData;
	    
	    // figure out the actual size of image array
	    int size = img_width * img_height * n_channels;
	    
	    uchar *temp[NUM_BGS];
	    
	    for (int i = 0; i < NUM_BGS; ++i)
	    {
	        temp[i] = (uchar *) bgs[i]->imageData;
	    }
	    
        for (int i = 0; i < size; ++i)
        {
            double sum = 0.0;
            
	        for (int j = 0; j < NUM_BGS; ++j)
            {
                sum += temp[j][i];
            }
            
            ave_data[i] = sum / (double) NUM_BGS;
        }
        
        std_dev.resize(size);
        
        for (int i = 0; i < size; ++i)
        {
            double sum = 0.0;
        
	        for (int j = 0; j < NUM_BGS; ++j)
            {
                 sum += pow(temp[j][i] - ave_data[i], 2.0);
            }
            
            std_dev[i] = sqrt(sum / (double) (NUM_BGS - 1));
        }
        
        for (int i = 0; i < NUM_BGS; ++i)
        {
            cvReleaseImage(&bgs[i]);
        }
        
        have_ave_bg = true;
	}
	else
	{
	    image_sub.shutdown();
	}
}

bool callback(background_filters::GetBgStats::Request& request, background_filters::GetBgStats::Response& response)
{
    sensor_msgs::CvBridge::fromIpltoRosImage(ave_bg, response.ave_bg);
    response.std_dev = std_dev;

    return true;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "background_sampler");
    ros::NodeHandle n;
    
    image_sub = n.subscribe("/camera/image_color", 1, handle_image);
    ave_bg_pub = n.advertise<sensor_msgs::Image>("average_bg", 1);
    
    service = n.advertiseService("get_bg_stats", callback);

    boost::thread t = boost::thread::thread(boost::bind(&publish_average_background));
    ros::spin();

    t.interrupt();
    t.join();
    
    return 0;
}

