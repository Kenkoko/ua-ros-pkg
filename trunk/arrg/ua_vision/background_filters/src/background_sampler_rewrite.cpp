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
#include <cv_bridge/CvBridge.h>
#include <boost/thread/thread.hpp>
#include <math.h>
#include <background_filters/GetBgStats.h>

#include <sys/types.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include "background_filters/common.h"

using namespace std;

class BackgroundAverager
{
private:
    double delay;
    int num_samples;
    double scale;
    string colorspace;

    int img_width;
    int img_height;
    int img_n_chan;

    cv::Mat avg_img;
    vector<cv::Mat> samples;
    vector<float> std_dev;
    vector<float> cov_mat;
    vector<float> cov_mat_inv;
    vector<float> dets;
    int sample_counter;
    bool have_avg_img;

    ros::ServiceServer avg_img_srv;
    ros::Subscriber image_sub;

public:
    BackgroundAverager(ros::NodeHandle& nh)
    {
        ros::NodeHandle local_nh("~");
        local_nh.param("scale", scale, 1.0);
        local_nh.param("colorspace", colorspace, string("rgb"));
        local_nh.param("number_of_samples", num_samples, 10);
        local_nh.param("sampling_delay", delay, 0.0);

        samples.resize(num_samples);;
        sample_counter = 0;
        have_avg_img = false;

        image_sub = nh.subscribe("image", 1, &BackgroundAverager::handle_image, this);
        avg_img_srv = nh.advertiseService("get_background_stats", &BackgroundAverager::get_background_stats, this);
    }

    void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        img_width = (int) (msg_ptr->width / scale);
        img_height = (int) (msg_ptr->height / scale);
        cv::Mat sample;

        if (sample_counter < num_samples)
        {
            sensor_msgs::CvBridge bridge;

            try
            {
                if (colorspace == "rgb")
                {
                    img_n_chan = 3;
                    sample = cv::Mat(img_height, img_width, CV_8UC3);
                    cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), sample, cv::Size(img_width, img_height));
                }
                else if (colorspace == "hsv")
                {
                    img_n_chan = 3;
                    sample = cv::Mat(img_height, img_width, CV_8UC3);
                    cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), sample, cv::Size(img_width, img_height));
                    cv::cvtColor(sample, sample, CV_BGR2HSV);
                }
                else if (colorspace == "rgchroma")
                {
                    img_n_chan = 2;
                    cv::Mat img(img_height, img_width, CV_8UC3);
                    cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), img, cv::Size(img_width, img_height));
                    sample = cv::Mat(img_height, img_width, CV_32FC2);
                    convertToChroma(img, sample);
                }

                samples[sample_counter++] = sample;

                if (delay > 0.0) { ros::Duration(delay).sleep(); }
            }
            catch (sensor_msgs::CvBridgeException error)
            {
                ROS_ERROR("CvBridgeError");
            }

            return;
        }

        if (!have_avg_img)
        {
            ROS_INFO("Collected samples for background averaging");
            ROS_INFO("Image: %dx%d with %d channels, requested colorspace is %s", img_width, img_height, img_n_chan, colorspace.c_str());

            if (colorspace == "rgb")
            {
                avg_img = cv::Mat(img_height, img_width, CV_8UC3);
                calculate_avg_img(1.0f);
            }
            if (colorspace == "hsv")
            {
                avg_img = cv::Mat(img_height, img_width, CV_8UC3);
                calculate_avg_img(50.0f);
            }
            else if (colorspace == "rgchroma")
            {
                avg_img = cv::Mat(img_height, img_width, CV_32FC2);
                calculate_avg_img(0.001f);
            }

            have_avg_img = true;
            ROS_INFO("Computed average background from samples");
        }
        else
        {
            image_sub.shutdown();
        }

    }

    void calculate_avg_img(float alpha)
    {
        int pixel_num = img_width * img_height;

        // a img_n_chan x img_n_chan covariance matrix for each pixel in the image
        cov_mat.resize(pixel_num * (img_n_chan * img_n_chan));
        cov_mat_inv.resize(pixel_num * (img_n_chan * img_n_chan));
        dets.resize(pixel_num);
        std_dev.resize(pixel_num * img_n_chan);

        int sample_img_type = -1;
        if (colorspace == "rgb" || colorspace == "hsv") { sample_img_type = CV_8UC1; }
        else if (colorspace == "rgchroma") { sample_img_type = CV_32FC1; }

        cv::Mat vects(num_samples, img_n_chan, sample_img_type);
        cv::Mat ave(1, img_n_chan, CV_32FC1);
        cv::Mat covMat(img_n_chan, img_n_chan, CV_32FC1);
        cv::Mat covMatInv(img_n_chan, img_n_chan, CV_32FC1);

        for (int row = 0; row < img_height; ++row)
        {
            for (int col = 0; col < img_width; ++col)
            {
                int pixel = row * img_width + col;

                for (int i = 0; i < num_samples; ++i)
                {
                    if (colorspace == "rgb" || colorspace == "hsv")
                    {
                        const uchar* ptr = samples[i].ptr<const uchar>(row);

                        for (int ch = 0; ch < img_n_chan; ++ch)
                        {
                            vects.at<uchar>(i, ch) = ptr[img_n_chan*col + ch];
                        }
                    }
                    else if (colorspace == "rgchroma")
                    {
                        const float* ptr = samples[i].ptr<const float>(row);

                        for (int ch = 0; ch < img_n_chan; ++ch)
                        {
                            vects.at<float>(i, ch) = ptr[img_n_chan*col + ch];
                        }
                    }
                }

                if (pixel == 0)
                {
                    for (int i = 0; i < num_samples; ++i)
                    {
                        CvMat m = vects.row(i);
                        print_mat(&m);
                    }
                }

                cv::calcCovarMatrix(vects, covMat, ave, CV_COVAR_NORMAL | CV_COVAR_SCALE | CV_COVAR_ROWS, CV_32F);

                if (pixel == 0)
                {
                    cout << "Averages of input vectors" << endl;
                    CvMat a = ave;
                    print_mat(&a);
                    cout << endl;

                    cout << endl << "Covariance matrix" << endl;
                    CvMat m = covMat;
                    print_mat(&m);
                    cout << endl;
                }

                // pad the diagonal of the covariance matrix to avoid 0's
                for (int ch = 0; ch < img_n_chan; ++ch)
                {
                    covMat.at<float>(ch, ch) += alpha;
                }

                if (pixel == 0)
                {
                    cout << "Covariance matrix after padding diagonal" << endl;
                    CvMat m = covMat;
                    print_mat(&m);
                    cout << endl;
                }

                dets[pixel] = cv::invert(covMat, covMatInv, cv::DECOMP_LU);

                if (pixel == 0)
                {
                    cout << "Covariance matrix after copying for transport" << endl;
                }

                // copy over the covariance matrix
                for (int r = 0; r < covMat.rows; ++r)
                {
                    const float* ptr = covMat.ptr<const float>(r);

                    for (int c = 0; c < covMat.cols; ++c)
                    {
                        cov_mat[pixel*img_n_chan*img_n_chan + r*covMat.cols + c] = *ptr++;

                        if (pixel == 0)
                        {
                            cout << cov_mat[pixel*img_n_chan*img_n_chan + r*covMat.cols + c] << " ";
                        }
                    }
                }

                if (pixel == 0)
                {
                    cout << endl;
                }

                // copy over the inverse covariance matrix
                for (int r = 0; r < covMatInv.rows; ++r)
                {
                    const float* ptr = covMatInv.ptr<const float>(r);

                    for (int c = 0; c < covMatInv.cols; ++c)
                    {
                        cov_mat_inv[pixel*img_n_chan*img_n_chan + r*covMatInv.cols + c] = *ptr++;
                    }
                }

                // copy over the average data
                for (int ch = 0; ch < img_n_chan; ++ch)
                {
                    float& f = ave.at<float>(0, ch);
                    if (colorspace == "rgb" || colorspace == "hsv") { avg_img.ptr<uchar>(row)[col*img_n_chan + ch] = f; }
                    else if (colorspace == "rgchroma") { avg_img.ptr<float>(row)[col*img_n_chan + ch] = f; }
                    //cout << "[" << pixel << ", " << ch << "] = " << f << " ~ " << (int) (avg_img.ptr<uchar>(row)[col*img_n_chan + ch]) << endl;
                }

                // get standard deviations from the diagonal of covariance matrix
                for (int ch = 0; ch < img_n_chan; ++ch)
                {
                    std_dev[pixel*img_n_chan + ch] = sqrt(covMat.at<float>(ch, ch));
                }
            }
        }

        samples.clear();
    }

    bool get_background_stats(background_filters::GetBgStats::Request& request, background_filters::GetBgStats::Response& response)
    {
        IplImage avg_img_ipl = avg_img;
        //cout << "In service" << endl;
        //print_img(&avg_img_ipl);
        response.colorspace = colorspace;
        sensor_msgs::CvBridge::fromIpltoRosImage(&avg_img_ipl, response.average_background);
        response.covariance_matrix = cov_mat;
        response.covariance_matrix_inv = cov_mat_inv;
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
    ros::spin();

    return 0;
}
