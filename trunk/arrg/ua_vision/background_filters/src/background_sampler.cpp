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

class BackgroundAverager
{
private:
    double delay;
    int num_samples;
    
    IplImage **bgs;
    IplImage *ave_bg;
    vector<double> std_dev;
    vector<double> cov_mat;
    vector<double> dets;
    int bg_counter;
    bool have_ave_bg;
    
    ros::ServiceServer service;
    ros::Subscriber image_sub;
    ros::Publisher ave_bg_pub;

public:
    BackgroundAverager(ros::NodeHandle& nh)
    {
        ros::NodeHandle ln("~");
        ln.param("number_of_samples", num_samples, 10);
        ln.param("sampling_delay", delay, 0.0);
        
        bgs = (IplImage **) calloc(num_samples, sizeof(IplImage *));
        bg_counter = 0;
        have_ave_bg = false;
        
        image_sub = nh.subscribe("image", 1, &BackgroundAverager::handle_image, this);
        ave_bg_pub = nh.advertise<sensor_msgs::Image>("average_bg", 1);
        service = nh.advertiseService("get_background_stats", &BackgroundAverager::get_background_stats, this);
    }
    
    void publish_average_background()
    {
        ros::Rate r(1.0);
        
        while (ros::ok())
        {
            if (have_ave_bg)
            {
                ave_bg_pub.publish(sensor_msgs::CvBridge::cvToImgMsg(ave_bg, "bgr8"));
                r.sleep();
            }
        }
    }

    void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        if (bg_counter < num_samples)
        {
            sensor_msgs::CvBridge bridge;
            IplImage *bg = NULL;
            
            try
            {
                bg = bridge.imgMsgToCv(msg_ptr, "bgr8");
                bgs[bg_counter++] = cvCloneImage(bg);
                if (delay > 0.0) { ros::Duration(delay).sleep(); }
            }
            catch (sensor_msgs::CvBridgeException error)
            {
                ROS_ERROR("CvBridgeError");
            }
            
            return;
        }
        
        if (!have_ave_bg)
        {
            ROS_INFO("Collected samples for background avergaing");
            
            int img_width = bgs[0]->width;
            int img_height = bgs[0]->height;
            int img_depth = bgs[0]->depth;
            int img_channels = bgs[0]->nChannels;
            
            ROS_INFO("Image: %dx%d with %d channels (depth is %d)", img_width, img_height, img_channels, img_depth);
            
            ave_bg = cvCreateImage(cvSize(img_width, img_height), img_depth, img_channels);
            uchar *ave_data = (uchar *) ave_bg->imageData;
            
            // figure out the actual size of image array
            int size = img_width * img_height * img_channels;
            
            uchar *temp[num_samples];
            
            for (int i = 0; i < num_samples; ++i)
            {
                temp[i] = (uchar *) bgs[i]->imageData;
            }
            
            // a 3x3 covariance matrix for each pixel in the image
            cov_mat.resize(img_width * img_height * 9);
            dets.resize(img_width * img_height);
            std_dev.resize(size);
            
            CvMat *ave = cvCreateMat(1, 3, CV_32FC1);
            CvMat *covMat = cvCreateMat(3, 3, CV_32FC1);
            CvMat **vects = (CvMat **) calloc(num_samples, sizeof(CvMat *));

            // for each pixel in the image
            for (int i = 0; i < size; i += 3)
            {
                // for each image sampled
                for (int j = 0; j < num_samples; ++j)
                {
                    if (i == 0) vects[j] = cvCreateMat(1, 3, CV_8UC1);
                    cvSet1D(vects[j], 0, cvScalar(temp[j][i]));
                    cvSet1D(vects[j], 1, cvScalar(temp[j][i+1]));
                    cvSet1D(vects[j], 2, cvScalar(temp[j][i+2]));
                }
                
                cvCalcCovarMatrix((const CvArr**) vects, num_samples, covMat, ave, CV_COVAR_NORMAL);
                dets[i/3] = cvDet(covMat);
                
                for (int row = 0; row < covMat->rows; ++row)
                {
                    const float* ptr = (const float*)(covMat->data.ptr + row * covMat->step);
                    
                    for (int col = 0; col < covMat->cols; ++col)
                    {
                        cov_mat[i + row*covMat->cols + col] = *ptr++;
                    }
                }
                
                for (int row = 0; row < ave->rows; ++row)
                {
                    const float* ptr = (const float*)(ave->data.ptr + row * ave->step);
                    
                    for (int col = 0; col < ave->cols; ++col)
                    {
                        ave_data[i + row*ave->cols + col] = *ptr++;
                    }
                }
                
                double sumb = 0.0;
                double sumg = 0.0;
                double sumr = 0.0;
                
                for (int j = 0; j < num_samples; ++j)
                {
                     sumb += pow(temp[j][i] - ave_data[i], 2.0);
                     sumg += pow(temp[j][i+1] - ave_data[i+1], 2.0);
                     sumr += pow(temp[j][i+2] - ave_data[i+2], 2.0);
                }
                
                std_dev[i] = sqrt(sumb / (double) (num_samples - 1));
                std_dev[i+1] = sqrt(sumg / (double) (num_samples - 1));
                std_dev[i+2] = sqrt(sumr / (double) (num_samples - 1));
            }
            
            for (int i = 0; i < num_samples; ++i)
            {
                cvReleaseMat(&vects[i]);
                temp[i] = NULL;
                cvReleaseImage(&bgs[i]);
            }
            
            cvReleaseMat(&ave);
            cvReleaseMat(&covMat);
            if (vects) free(vects);
            if (bgs) free(bgs);
            
            have_ave_bg = true;
            ROS_INFO("Computed average background from samples");
        }
        else
        {
            image_sub.shutdown();
        }
    }

    bool get_background_stats(background_filters::GetBgStats::Request& request, background_filters::GetBgStats::Response& response)
    {
        sensor_msgs::CvBridge::fromIpltoRosImage(ave_bg, response.average_background);
        response.covariance_matrix = cov_mat;
        response.covariance_matrix_dets = dets;
        response.standard_deviations = std_dev;
        
        return true;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "background_sampler");
    ros::NodeHandle n;
    
    BackgroundAverager ba(n);
    boost::thread t = boost::thread::thread(boost::bind(&BackgroundAverager::publish_average_background, &ba));
    ros::spin();
    
    t.interrupt();
    t.join();
    
    return 0;
}

