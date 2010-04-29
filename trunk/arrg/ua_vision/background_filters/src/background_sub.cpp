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
uchar *ave_data;

vector<float> cov_mats;
vector<float> cov_mats_inv;
vector<float> dets;
vector<float> std_dev;

ros::Subscriber image_sub;
ros::Publisher prob_img_pub;

void print_mat(CvMat *A)
{
    int i, j;
    for (i = 0; i < A->rows; i++)
    {
        printf("\n");
        switch (CV_MAT_DEPTH(A->type))
        {
            case CV_32F:
            case CV_64F:
                for (j = 0; j < A->cols; j++)
                printf ("%8.3f ", (float)cvGetReal2D(A, i, j));
                break;
            case CV_8U:
            case CV_16U:
                for(j = 0; j < A->cols; j++)
                printf ("%6d",(int)cvGetReal2D(A, i, j));
                break;
            default:
            break;
        }
    }
    printf("\n");
}

bool first = true;

void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
{
    sensor_msgs::CvBridge bridge;
    IplImage *bg = NULL;

    try
    {
        bg = cvCreateImage(cvGetSize(ave_bg), IPL_DEPTH_8U, ave_bg->nChannels);
        cvResize(bridge.imgMsgToCv(msg_ptr, "bgr8"), bg);
    }
    catch (sensor_msgs::CvBridgeException error)
    {
        ROS_ERROR("CvBridgeError");
    }
    
    uchar *bg_data = (uchar *) bg->imageData;
    
    IplImage *prob_img = cvCreateImage(cvGetSize(bg), IPL_DEPTH_32F, 1);
    float *prob_data = (float *) prob_img->imageData;

    int width = bg->width;
    int height = bg->height;
    
    for (int y = 0; y < height; ++y)
    {
        uchar* ptr_bg = bg_data + y * bg->widthStep;
        uchar* ptr_ave = ave_data + y * ave_bg->widthStep;
        
        for (int x = 0; x < width; ++x)
        {
            CvMat *bgr_new = cvCreateMat(1, 3, CV_32FC1);
            CvMat *bgr_ave = cvCreateMat(1, 3, CV_32FC1);
            CvMat inv_cov;
            
            cvSet1D(bgr_new, 0, cvScalar(ptr_bg[3*x]));
            cvSet1D(bgr_new, 1, cvScalar(ptr_bg[3*x+1]));
            cvSet1D(bgr_new, 2, cvScalar(ptr_bg[3*x+2]));

            cvSet1D(bgr_ave, 0, cvScalar(ptr_ave[3*x]));
            cvSet1D(bgr_ave, 1, cvScalar(ptr_ave[3*x+1]));
            cvSet1D(bgr_ave, 2, cvScalar(ptr_ave[3*x+2]));

            cvInitMatHeader(&inv_cov, 3, 3, CV_32FC1, &cov_mats_inv[(y*width+x)*9]);
            
            double mah_dist = cvMahalanobis(bgr_new, bgr_ave, &inv_cov);
            double unnorm_gaussian = exp(-0.5 * mah_dist);        
            double partition = 1.0 / (pow(TWO_PI, 1.5) * sqrt(dets[y*width+x]));
            float p = partition * unnorm_gaussian;

            prob_data[y*width+x] = p;
            
            cvReleaseMat(&bgr_new);
            cvReleaseMat(&bgr_ave);
        }
    }

    double min, max;
    cvMinMaxLoc(prob_img, &min, &max);
    cvConvertScale(prob_img, prob_img, 1.0 / max);

    cvShowImage("prob_img", prob_img);
    cvWaitKey(100);
//    if (first)
//    {
//        CvMat* m = cvCreateMatHeader(prob_img->height, prob_img->width, CV_32FC1);
//        cvGetMat(prob_img, m);
//        print_mat(m);
//        first = false;
//    }
    prob_img_pub.publish(sensor_msgs::CvBridge::cvToImgMsg(prob_img));
    
    cvReleaseImage(&bg);
    bg_data = NULL;
    
    cvReleaseImage(&prob_img);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "background_sub");
    ros::NodeHandle n;
    
    ros::ServiceClient client = n.serviceClient<background_filters::GetBgStats>("get_background_stats");
    
    background_filters::GetBgStats srv;
    sensor_msgs::CvBridge b;

    cvNamedWindow("prob_img");

    if (client.call(srv))
    {
        cout << "calling service" << endl;

        b.fromImage(srv.response.average_background, "bgr8");
        ave_bg = b.toIpl();
        ave_data = (uchar *) ave_bg->imageData;
        cov_mats = srv.response.covariance_matrix;
        cov_mats_inv = srv.response.covariance_matrix_inv;
        dets = srv.response.covariance_matrix_dets;
        std_dev = srv.response.standard_deviations;

        cout << "done calling service" << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service get_bg_stats");
        return 1;
    }
    
    cout << "subscribing" << endl;
    image_sub = n.subscribe("image", 1, handle_image);
    ros::Publisher ave_bg_pub = n.advertise<sensor_msgs::Image>("average_bg_service", 1);
    prob_img_pub = n.advertise<sensor_msgs::Image>("prob_img", 1);
    cout << "done subscribing" << endl;

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
