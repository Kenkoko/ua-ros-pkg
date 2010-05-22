/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Antons Rebguns.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <geometry_msgs/Point.h>

#include <boost/thread.hpp>
#include <boost/format.hpp>

#include <nmpt/BlockTimer.h>
#include <nmpt/FastSaliency.h>

#include <cv_bridge/CvBridge.h>
#include <math.h>
#include <background_filters/GetBgStats.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include "background_filters/common.h"

#define TWO_PI 6.28318531

using namespace std;

class SaliencyTracker
{
private:
    std::string window_name;
    CvFont font;

    IplImage* small_color_image;
    IplImage* small_float_image;
    IplImage* composite_image;

    //The timer is used to track the frame rate
    BlockTimer timer;
    FastSaliency* saltracker;

    // FastSUN saliency tracker parameters
    int sp_scales;
    int tm_scales;
    int tau0;
    int rad0;
    int doeFeat;
    int dobFeat;
    int colorFeat;
    int useParams;
    int estParams;
    int power;

    int imwidth;
    int imheight;

    // Parameter constraints
    int maxPowers;
    int maxScales;
    int maxRad;
    int maxTau;

    bool initialized;

public:
    SaliencyTracker(ros::NodeHandle& local_nh)
    {
        window_name = "saliency";
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, .33, .33);

        saltracker = NULL;

        // Parameter constraints
        maxPowers = 19;
        maxScales = 6;
        maxRad = 3;
        maxTau = 10;

        // FastSUN saliency tracker parameters
        local_nh.param("spatial_scales", sp_scales, 5);
        local_nh.param("temporal_scales", tm_scales, 0);
        local_nh.param("spatial_size", rad0, 0);
        local_nh.param("temporal_falloff", tau0, 0);
        local_nh.param("distribution_power", power, 9);
        local_nh.param("use_spatial_features", dobFeat, 1);
        local_nh.param("use_temporal_features", doeFeat, 1);
        local_nh.param("use_color_contrast", colorFeat, 1);
        local_nh.param("estimate_histogram", estParams, 1);
        local_nh.param("use_histogram", useParams, 1);

        if (sp_scales > maxScales) { sp_scales = maxScales; }
        else if (sp_scales < 0) { sp_scales = 0; }

        if (tm_scales > maxScales) { tm_scales = maxScales; }
        else if (tm_scales < 0) { tm_scales = 0; }

        if (rad0 > maxRad) { rad0 = maxRad; }
        else if (rad0 < 0) { rad0 = 0; }

        if (tau0 > maxTau) { tau0 = maxTau; }
        else if (tau0 < 0) { tau0 = 0; }

        if (power > maxPowers) { power = maxPowers; }
        else if (power < 0) { power = 0; }

        initialized = false;
    }

    ~SaliencyTracker()
    {
        cvDestroyWindow(window_name.c_str());
        cvReleaseImage(&small_color_image);
        cvReleaseImage(&small_float_image);
        cvReleaseImage(&composite_image);
        delete saltracker;
    }

    void init(IplImage* img)
    {
        imwidth = img->width;
        imheight = img->height;

        ROS_INFO("Initializing saliency tracker");
        ROS_INFO("Saliency image size is (%d, %d)", imwidth, imheight);

        int nspatial = sp_scales + 1;       // [2 3 4 5 ...]
        int ntemporal = tm_scales + 2;      // [2 3 4 5 ...]
        float first_tau = 1.0 / (tau0 + 1); // [1 1/2 1/3 1/4 1/5 ...]
        int first_rad = (1 << rad0) / 2;    // [0 1 2 4 8 16 ...]

        saltracker = new FastSaliency(imwidth, imheight, ntemporal, nspatial, first_tau, first_rad);

        double mypower = 0.1 + power / 10.0;
        saltracker->setGGDistributionPower(mypower);
        saltracker->setUseDoEFeatures(doeFeat);
        saltracker->setUseDoBFeatures(dobFeat);
        saltracker->setUseColorInformation(colorFeat);
        saltracker->setUseGGDistributionParams(useParams);
        saltracker->setEstimateGGDistributionParams(estParams);

        //Make some intermediate image representations:
        //(1) The downsized representation of the original image frame
        small_color_image = cvCreateImage(cvSize(imwidth, imheight), img->depth, img->nChannels);

        //(2) The floating point image that is passed to the saliency algorithm; this also
        //doubles as an intermediate representation for the output.
        small_float_image = cvCreateImage(cvSize(imwidth, imheight), IPL_DEPTH_32F, img->nChannels);

        //(3) An image to visualize both the original image and the saliency image simultaneously
        composite_image = cvCreateImage(cvSize(imwidth * 2, imheight), img->depth, img->nChannels);

        timer.blockStart(0);
        cvNamedWindow(window_name.c_str(), CV_WINDOW_AUTOSIZE);
        initialized = true;
    }

    void process_image(IplImage* img)
    {
        //Put the current frame into the format expected by the saliency algorithm
        cvResize(img, small_color_image, CV_INTER_LINEAR);
        cvConvert(small_color_image, small_float_image);

        //Call the "updateSaliency" method, and time how long it takes to run
        timer.blockStart(1);
        saltracker->updateSaliency(small_float_image);
        timer.blockStop(1);

        //Paste the original color image into the left half of the display image
        CvRect half = cvRect(0, 0, imwidth, imheight);
        cvSetImageROI(composite_image, half);
        cvCopy(small_color_image, composite_image, NULL);

        cvCvtColor(saltracker->salImageFloat, small_float_image, CV_GRAY2BGR);

        // Paste the saliency map into the right half of the color image
        half = cvRect(imwidth, 0, imwidth, imheight);
        cvSetImageROI(composite_image, half);
        cvConvertScale(small_float_image, composite_image, 255, 0);
        cvResetImageROI(composite_image);

        //on some systems, the image is upside down
        if( img->origin != IPL_ORIGIN_TL ) { cvFlip(composite_image, NULL, 0); }

        timer.blockStop(0);

        double tot = timer.getTotTime(0);
        double fps = timer.getTotTime(1);

        timer.blockReset(0);
        timer.blockReset(1);
        timer.blockStart(0);

        //Print the timing information to the display image
        char str[1000] = {'\0'};
        sprintf(str,"FastSUN: %03.1f MS;   Total: %03.1f MS", 1000.0*fps, 1000.0*tot);
        cvPutText(composite_image, str, cvPoint(20,20), &font, CV_RGB(255,0,255));

        cvShowImage(window_name.c_str(), composite_image);
    }
};

class BackgroundSubtractor
{
private:
    cv::Mat avg_img;

    vector<float> cov_mats;
    vector<float> cov_mats_inv;
    vector<float> dets;
    vector<float> std_dev;
    vector<double> partition;

    int img_n_chan;
    string colorspace;

    SaliencyTracker* sal_tracker;
    ros::Subscriber image_sub;
    ros::Publisher prob_img_pub;

    //boost::function<void (const cv::Mat&)> diff_func;

public:
    BackgroundSubtractor(ros::NodeHandle& nh)
    {
        ros::ServiceClient client = nh.serviceClient<background_filters::GetBgStats>("get_background_stats");

        background_filters::GetBgStats srv;
        sensor_msgs::CvBridge bridge;

        //diff_func = boost::bind(&BackgroundSubtractor::difference, this, _1);

        cvNamedWindow("prob_img");

        if (client.call(srv))
        {
            bridge.fromImage(srv.response.average_background);
            avg_img = cv::Mat(bridge.toIpl()).clone();
            colorspace = srv.response.colorspace;
            cov_mats = srv.response.covariance_matrix;
            cov_mats_inv = srv.response.covariance_matrix_inv;
            dets = srv.response.covariance_matrix_dets;
            std_dev = srv.response.standard_deviations;

            img_n_chan = avg_img.channels();

            partition.resize(dets.size());
            double coef = pow(TWO_PI, img_n_chan / 2.0);

            for (unsigned i = 0; i < dets.size(); ++i)
            {
                partition[i] = 1.0 / (coef * sqrt(dets[i]));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service get_bg_stats");
        }

        IplImage avg_img_ipl = avg_img;

        if (colorspace == "rgb" || colorspace == "hsv")
        {
            ros::NodeHandle local_nh = ros::NodeHandle("~");
            sal_tracker = new SaliencyTracker(local_nh);
            sal_tracker->init(&avg_img_ipl);
        }

        image_sub = nh.subscribe("image", 1, &BackgroundSubtractor::handle_image, this);
        prob_img_pub = nh.advertise<sensor_msgs::Image>("probability_image", 1);
    }

    ~BackgroundSubtractor()
    {
        cvDestroyWindow("prob_img");
        if (colorspace == "rgb" || colorspace == "hsv") { delete sal_tracker; }
    }

    void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        sensor_msgs::CvBridge bridge;
        cv::Mat new_img;

        try
        {
            if (colorspace == "rgb")
            {
                new_img = cv::Mat(avg_img.size(), CV_8UC3);
                cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), new_img, avg_img.size());
            }
            else if (colorspace == "hsv")
            {
                new_img = cv::Mat(avg_img.size(), CV_8UC3);
                cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), new_img, avg_img.size());
                cv::cvtColor(new_img, new_img, CV_BGR2HSV);
            }
            else if (colorspace == "rgchroma")
            {
                cv::Mat temp(avg_img.size(), CV_8UC3);
                cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), temp, avg_img.size());
                new_img = cv::Mat(avg_img.size(), CV_32FC2);
                convertToChroma(temp, new_img);
            }
        }
        catch (sensor_msgs::CvBridgeException error)
        {
            ROS_ERROR("CvBridgeError");
        }

        if (colorspace == "rgb" || colorspace == "hsv")
        {
            IplImage new_img_ipl = new_img;
            sal_tracker->process_image(&new_img_ipl);
            difference<const uchar>(new_img);
        }
        else if (colorspace == "rgchroma")
        {
            difference<const float>(new_img);
        }

        //diff_func(new_img);
        //boost::thread diff_thread = boost::thread(diff_func, new_img);
        //diff_thread.join();
    }

    template <class T>
    void difference(const cv::Mat& new_img)
    {
        cv::Mat prob_img(new_img.size(), CV_32FC1);
        float *prob_data = prob_img.ptr<float>();

        int height = new_img.rows;
        int width = new_img.cols;

        cv::Mat bgr_new(1, img_n_chan, CV_32FC1);
        cv::Mat bgr_ave(1, img_n_chan, CV_32FC1);
        cv::Mat inv_cov;

        for (int row = 0; row < height; ++row)
        {
            T* ptr_bg = new_img.ptr<T>(row);
            T* ptr_ave = avg_img.ptr<T>(row);

            for (int col = 0; col < width; ++col)
            {
                int pixel = row * width + col;

                for (int ch = 0; ch < img_n_chan; ++ch)
                {
                    bgr_new.at<float>(0, ch) = ptr_bg[img_n_chan*col + ch];
                    bgr_ave.at<float>(0, ch) = ptr_ave[img_n_chan*col + ch];
                }

                inv_cov = cv::Mat(img_n_chan, img_n_chan, CV_32FC1, &cov_mats_inv[pixel*(img_n_chan*img_n_chan)], sizeof(float)*img_n_chan);

                double mah_dist = cv::Mahalanobis(bgr_new, bgr_ave, inv_cov);
                double unnorm_gaussian = exp(-0.5 * mah_dist);
                float p = partition[pixel] * unnorm_gaussian;

                prob_data[pixel] = p;
            }
        }

        // calculate negative log-likelihood, darker areas are background, lighter - not
        double min, max;
        cv::log(prob_img, prob_img);
        cv::minMaxLoc(prob_img, &min, &max);
        prob_img.convertTo(prob_img, prob_img.type(), -1.0 / (max - min), 1 + min / (max - min));

        cv::imshow("prob_img", prob_img);

        IplImage prob_img_ipl = prob_img;
        prob_img_pub.publish(sensor_msgs::CvBridge::cvToImgMsg(&prob_img_ipl));
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "background_subtractor");
    ros::NodeHandle n;

    cvStartWindowThread();
    BackgroundSubtractor subtractor(n);
    ros::spin();

    return 0;
}
