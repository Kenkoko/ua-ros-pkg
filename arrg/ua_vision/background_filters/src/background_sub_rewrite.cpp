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
#include <boost/foreach.hpp>

#include <nmpt/BlockTimer.h>
#include <nmpt/FastSaliency.h>

#include <cv_bridge/CvBridge.h>
#include <math.h>
#include <background_filters/GetBgStats.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <nmpt/OpenCVBoxFilter.h>

#include "background_filters/common.h"

#define TWO_PI 6.28318531
#include <boost/concept_check.hpp>

using namespace std;

class SaliencyTracker
{
public:
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

    float g_min;
    float g_max;

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

        g_min = numeric_limits<float>::max();
        g_max = numeric_limits<float>::min();

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

    cv::Mat process_image(IplImage* img)
    {
        //Put the current frame into the format expected by the saliency algorithm
        cvResize(img, small_color_image, CV_INTER_LINEAR);
        cvConvert(small_color_image, small_float_image);

        //Call the "updateSaliency" method, and time how long it takes to run
        timer.blockStart(1);
        saltracker->updateSaliency(small_float_image);
        timer.blockStop(1);

        // copy over the saliency map before we further modify it,
        // values in result matrix are unnormalized negative log likelihoods
        cv::Mat result = cv::Mat(saltracker->salImageFloat).clone();

        //Paste the original color image into the left half of the display image
        CvRect half = cvRect(0, 0, imwidth, imheight);
        cvSetImageROI(composite_image, half);
        cvCopy(small_color_image, composite_image, NULL);

        double min, max;
        cvMinMaxLoc(saltracker->salImageFloat, &min, &max);
        if (min < g_min) { g_min = min; }
        if (max > g_max) { g_max = max; }
        cvConvertScale(saltracker->salImageFloat, saltracker->salImageFloat, 1.0 / (g_max - g_min), -g_min / (g_max - g_min));
        //cvNormalize(saltracker->salImageFloat, saltracker->salImageFloat, 0, 1, CV_MINMAX, NULL);
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
        return result;
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

    OpenCVBoxFilter* box_filter;
    vector<int> box_size;
    vector<cv::Rect> box_position;
    vector<cv::Mat> filtered;

    float g_min;
    float g_max;

    int counter;

    //boost::function<void (const cv::Mat&)> diff_func;

public:
    BackgroundSubtractor(ros::NodeHandle& nh)
    {
        ros::ServiceClient client = nh.serviceClient<background_filters::GetBgStats>("get_background_stats");

        background_filters::GetBgStats srv;
        sensor_msgs::CvBridge bridge;

        g_min = numeric_limits<float>::max();
        g_max = numeric_limits<float>::min();

        counter = 0;

        //diff_func = boost::bind(&BackgroundSubtractor::difference, this, _1);

        cvNamedWindow("prob_img");
        cvNamedWindow("combined");
        cvNamedWindow("filtered");
        cvNamedWindow("binary");
        cvNamedWindow("contours");
        cvNamedWindow("fg_prob_img");

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

            box_size.push_back(1);

            for (int i = 1; i <= 4; ++i)
            {
                box_size.push_back(2 * box_size[i-1]);
            }

            for (int i = 0; i <= 4; ++i)
            {
                box_size[i] = 2 * box_size[i] + 1;
                box_position.push_back(cv::Rect(-box_size[i]/2, -box_size[i]/2, box_size[i], box_size[i]));
                filtered.push_back(cv::Mat::zeros(avg_img.size(), CV_64FC1));
            }

            box_filter = new OpenCVBoxFilter(avg_img.cols, avg_img.rows, box_size[4] / 2);
        }

        image_sub = nh.subscribe("camera/image_color", 1, &BackgroundSubtractor::handle_image, this);
        prob_img_pub = nh.advertise<sensor_msgs::Image>("probability_image", 1);
    }

    ~BackgroundSubtractor()
    {
        cvDestroyWindow("prob_img");

        if (colorspace == "rgb" || colorspace == "hsv")
        {
            delete sal_tracker;
            delete box_filter;
        }
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
            if (counter > 0)
            {
                sal_tracker->saltracker->setEstimateGGDistributionParams(false);
                sal_tracker->saltracker->setUseGGDistributionParams(true);
            }
            else
            {
                IplImage new_img_ipl = new_img;
                sal_tracker->process_image(&new_img_ipl);
                ++counter;
                return;
            }

            // add negative log likelihoods obtained from probabilistic
            // background subtraction and fast saliency algorithm
            IplImage new_img_ipl = new_img;
            cv::Mat result = //sal_tracker->process_image(&new_img_ipl) +
                             difference<const uchar>(new_img);

            //print_img(&((IplImage) result));

            double w = -1/20.0, b = 1;
            cv::Mat fg_prob_img = result.clone();
            fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), w, b);
            cv::exp(fg_prob_img, fg_prob_img);
            fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), 1, 1);
            cv::divide(1.0, fg_prob_img, fg_prob_img);
            cv::imshow("fg_prob_img", fg_prob_img);

            double sum = cv::sum(fg_prob_img)[0];

            // use box filter for smoothing
            IplImage in = fg_prob_img;
            box_filter->setNewImage(&in);

            // create a bunch of images filtered with different sized box filters
            for (int i = 0; i <= 4; ++i)
            {
                IplImage out = filtered[i];
                box_filter->setBoxFilter(&out, box_position[i], 1.0/(box_position[i].width*box_position[i].height));
                double h = box_size[i] * box_size[i];
                filtered[i].convertTo(filtered[i], filtered[i].type(), -2*h, sum+h);
                //filtered[i].convertTo(filtered[i], filtered[i].type(), 1.0/(filtered[i].rows*filtered[i].cols));
            }

            double current_best = numeric_limits<double>::max();
            int current_best_id = -1;
            cv::Point current_best_point;

            for (int i = 0; i <= 4; ++i)
            {
                double min, max;
                cv::Point min_p;
                cv::minMaxLoc(filtered[i], &min, &max, &min_p);

                double t = -((min - sum)/(box_size[i] * box_size[i]) - 1)/2.0;
                if (t > 0.8 && min < current_best)
                {
                    current_best = min;
                    current_best_id = i;
                    current_best_point = min_p;
                }
            }

            for (int i = 0; i <= 4; ++i)
            {
                int hb = box_size[i] / 2;
                for (int row = hb + 1; row < filtered[i].rows - hb - 1; ++row)
                {
                    for (int col = hb + 1; col < filtered[i].cols - hb - 1; ++col)
                    {
                        const double* ptr = filtered[i].ptr<const double>(row);
                        if (ptr[col] < 0.32)
                        {
                            //printf("row = %d, col = %d, val = %g\n", row, col, ptr[col]);
                            int x = col - hb;
                            int y = row - hb;
                            cv::Rect roi(x, y, box_size[i], box_size[i]);
                            cv::rectangle(new_img, roi, cv::Scalar(255, 0, 0));
                        }
                    }
                }
            }

            //if (min < g_min) { g_min = min; }
            //if (max > g_max) { g_max = max; }
            //filtered.convertTo(filtered, filtered.type(), 1.0 / (g_max - g_min), -g_min / (g_max - g_min));
            //cv::normalize(filtered[3], filtered[3], 0.0, 1.0, cv::NORM_MINMAX);
            cv::normalize(result, result, 0.0, 1.0, cv::NORM_MINMAX);

            //cv::Mat f8;
            //filtered[3].convertTo(f8, CV_8U, 255.0);
            //cv::threshold(f8, f8, 130, 255, cv::THRESH_BINARY);

            cv::imshow("combined", result);
            //cv::imshow("filtered", filtered[3]);
            //cv::imshow("binary", f8);

/*            vector<vector<cv::Point> > contours;
            cv::findContours(f8, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            vector<vector<cv::Point> > contours_to_draw;

            BOOST_FOREACH(vector<cv::Point> contour, contours)
            {
                double area = fabs(cv::contourArea(cv::Mat(contour)));

                if (area > 250)
                {
                    cv::RotatedRect min_box = cv::minAreaRect(cv::Mat(contour));
                    cv::Rect bound_box = cv::boundingRect(cv::Mat(contour));
                    cv::rectangle(new_img, bound_box, cv::Scalar(255, 0, 0));
                    contours_to_draw.push_back(contour);
                }
            }

            cv::Scalar color(rand()&255, rand()&255, rand()&255);
            cv::drawContours(new_img, contours_to_draw, -1, color, 1);*/
            cv::imshow("contours", new_img);
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
    cv::Mat difference(const cv::Mat& new_img)
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

        // calculate negative log-likelihood, darker areas are background, lighter - objects
        cv::log(prob_img, prob_img);
        prob_img.convertTo(prob_img, prob_img.type(), -1.0);

        cv::Mat result = prob_img.clone();
        cv::normalize(prob_img, prob_img, 0.0, 1.0, cv::NORM_MINMAX);

        IplImage prob_img_ipl = prob_img;
        prob_img_pub.publish(sensor_msgs::CvBridge::cvToImgMsg(&prob_img_ipl));
        cv::imshow("prob_img", prob_img);

        return result;
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
