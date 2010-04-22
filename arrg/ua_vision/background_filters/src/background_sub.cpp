#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <boost/thread/thread.hpp>
#include <math.h>
#include <background_filters/GetBgStats.h>

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#define TWO_PI 6.28318531

using namespace std;

IplImage *ave_bg;
uchar *ave_img;

vector<double> cov_mats;
vector<double> dets;
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
    
    IplImage *prob_img = cvCreateImage(cvGetSize(bg), IPL_DEPTH_64F, 1);

    for (int i = 0; i < bg->imageSize; ++i)
    {
        double sum = 0.0;
        
        int px_new = (uchar) new_img[i];
        int px_ave = (uchar) ave_img[i];
        
        double sd = (std_dev[i] < 0.1) ? 0.1 : std_dev[i];
        double scale = 1.0 / sqrt(TWO_PI * sd);
        double p = scale * exp(-0.5 * pow((px_new - px_ave) / sd, 2.0));
    }
    
    new_img = NULL;
    
    cvReleaseImage(&prob_img);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "background_sub");
    ros::NodeHandle n;
    
    image_sub = n.subscribe("image", 1, handle_image);
    ros::ServiceClient client = n.serviceClient<background_filters::GetBgStats>("get_background_stats");
    ros::Publisher ave_bg_pub = n.advertise<sensor_msgs::Image>("average_bg_service", 1);
    
    background_filters::GetBgStats srv;
    sensor_msgs::CvBridge b;

    if (client.call(srv))
    {
        b.fromImage(srv.response.average_background, "bgr8");
        ave_bg = b.toIpl();
        ave_img = (uchar *) ave_bg->imageData;
        cov_mats = srv.response.covariance_matrix;
        dets = srv.response.covariance_matrix_dets;
        std_dev = srv.response.standard_deviations;
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
