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

vector<float> cov_mats;
vector<float> cov_mats_inv;
vector<float> dets;
vector<float> std_dev;

ros::Subscriber image_sub;
ros::Publisher prob_img_pub;

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
    
    IplImage *prob_img = cvCreateImage(cvGetSize(bg), IPL_DEPTH_32F, 1);
    float *prob_data = (float *) prob_img->imageData;

    int width = bg->width;
    int height = bg->height;

    for (int i = 0; i < width*height; ++i)
    {
        CvMat *bgr_new = cvCreateMat(1, 3, CV_32FC1);
        CvMat *bgr_ave = cvCreateMat(1, 3, CV_32FC1);
        CvMat inv_cov;
        
        cvSet1D(bgr_new, 0, cvScalar(new_img[i*3]));
        cvSet1D(bgr_new, 1, cvScalar(new_img[i*3+1]));
        cvSet1D(bgr_new, 2, cvScalar(new_img[i*3+2]));

        cvSet1D(bgr_ave, 0, cvScalar(ave_img[i*3]));
        cvSet1D(bgr_ave, 1, cvScalar(ave_img[i*3+1]));
        cvSet1D(bgr_ave, 2, cvScalar(ave_img[i*3+2]));

        cvInitMatHeader(&inv_cov, 3, 3, CV_32FC1, &cov_mats_inv[i*9]);
        
        double mah_dist = cvMahalanobis(bgr_new, bgr_ave, &inv_cov);
        double unnorm_gaussian = exp(-0.5 * mah_dist);        
        double partition = 1.0 / (pow(TWO_PI, 1.5) * sqrt(dets[i]));

        //cout << "mah = " << mah_dist << ", guss = " << unnorm_gaussian << ", part = " << partition << endl;
        float p = partition * unnorm_gaussian;

//        if (p > 1 || p < 0)
        {
            cout << i << " : " << p << endl;
        }

        prob_data[i] = p;

        cvReleaseMat(&bgr_new);
        cvReleaseMat(&bgr_ave);
    }

    cvShowImage("prob_img", prob_img);
    cvWaitKey(1000);
    prob_img_pub.publish(sensor_msgs::CvBridge::cvToImgMsg(prob_img));
    
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
    prob_img_pub = n.advertise<sensor_msgs::Image>("prob_img", 1);
    
    background_filters::GetBgStats srv;
    sensor_msgs::CvBridge b;

    cvNamedWindow("prob_img");

    if (client.call(srv))
    {
        b.fromImage(srv.response.average_background, "bgr8");
        ave_bg = b.toIpl();
        ave_img = (uchar *) ave_bg->imageData;
        cov_mats = srv.response.covariance_matrix;
        cov_mats_inv = srv.response.covariance_matrix_inv;
        dets = srv.response.covariance_matrix_dets;
        std_dev = srv.response.standard_deviations;
    }
    else
    {
        ROS_ERROR("Failed to call service get_bg_stats");
        return 1;
    }
    
//    ros::Rate r(1.0);
//    while (ros::ok())
//    {
//        ave_bg_pub.publish(b.cvToImgMsg(ave_bg, "bgr8"));
//        ros::spinOnce();
//        r.sleep();
//    }

    ros::spin();

    return 0;
}
