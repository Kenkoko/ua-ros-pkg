#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <boost/thread/thread.hpp>
#include <math.h>
#include <background_filters/GetBgStats.h>

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

using namespace std;

IplImage *ave_bg;
vector<double> std_dev;

ros::Subscriber image_sub;

void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
{
    sensor_msgs::CvBridge bridge;
    IplImage *bg = NULL;

    try
    {
        bg = bridge.imgMsgToCv(msg_ptr, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
	    ROS_ERROR("CvBridgeError");
    }
    
    uchar *new_img = (uchar *) bg->imageData;
    uchar *ave_img = (uchar *) ave_img->imageData;
    
    IplImage *prob_img = cvCreateImage(cvGetSize(bg), IPL_DEPTH_64F, 1);

    for (int i = 0; i < bg->imageSize; ++i)
    {
        double sum = 0.0;
        
        int px_new = (uchar) new_img[i];
        int px_ave = (uchar) ave_img[i];
        
        double scale = 1.0 / sqrt(2.0 * 3.14 * std_dev[i]);
        double sd = (std_dev[i] < 0.1) ? 0.1 : std_dev[i];
        double p = scale * exp(-0.5 * pow((px_new - px_ave) / sd, 2.0));
    }
    
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "background_sub");
    ros::NodeHandle n;
    
    image_sub = n.subscribe("/camera/image_color", 1, handle_image);
    ros::ServiceClient client = n.serviceClient<background_filters::GetBgStats>("get_bg_stats");
    ros::Publisher ave_bg_pub = n.advertise<sensor_msgs::Image>("average_bg_service", 1);
    
    background_filters::GetBgStats srv;
    sensor_msgs::CvBridge b;

    if (client.call(srv))
    {
        b.fromImage(srv.response.ave_bg, "bgr8");
        ave_bg = b.toIpl();
        std_dev = srv.response.std_dev;
        ROS_ERROR("YEAH!");
    }
    else
    {
        ROS_ERROR("Failed to call service get_bg_stats");
        return 1;
    }
    
    
    
    
    ros::Rate r(1.0);
    while (ros::ok())
    {
        ave_bg_pub.publish(b.cvToImgMsg(ave_bg, "bgr8"));
        r.sleep();
    }

    //ros::spin();

    return 0;
}
