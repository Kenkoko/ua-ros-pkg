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

#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

#include "background_filters/common.h"

#define TWO_PI 6.28318531

using namespace std;

class BackgroundSubtractor
{
private:
    cv::Mat avg_img;

    vector<float> cov_mats;
    vector<float> cov_mats_inv;
    vector<float> dets;
    vector<float> std_dev;

    int img_n_chan;
    string colorspace;
    bool first;

    ros::Subscriber image_sub;
    ros::Publisher prob_img_pub;

public:
    BackgroundSubtractor(ros::NodeHandle& nh)
    {
        ros::ServiceClient client = nh.serviceClient<background_filters::GetBgStats>("get_background_stats");

        first = true;

        background_filters::GetBgStats srv;
        sensor_msgs::CvBridge bridge;

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
        }
        else
        {
            ROS_ERROR("Failed to call service get_bg_stats");
        }

        image_sub = nh.subscribe("image", 1, &BackgroundSubtractor::handle_image, this);
        prob_img_pub = nh.advertise<sensor_msgs::Image>("probability_image", 1);
    }

    void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        sensor_msgs::CvBridge bridge;
        cv::Mat new_img;

        try
        {
            if (colorspace == "rgb")
            {
                img_n_chan = 3;
                new_img = cv::Mat(avg_img.size(), CV_8UC3);
                cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), new_img, avg_img.size());
            }
            else if (colorspace == "hsv")
            {
                img_n_chan = 3;
                new_img = cv::Mat(avg_img.size(), CV_8UC3);
                cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), new_img, avg_img.size());
                cv::cvtColor(new_img, new_img, CV_BGR2HSV);
            }
            else if (colorspace == "rgchroma")
            {
                img_n_chan = 2;
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

        cv::Mat prob_img(new_img.size(), CV_32FC1);
        float *prob_data = prob_img.ptr<float>();

        int height = new_img.rows;
        int width = new_img.cols;

        cv::Mat bgr_new(1, img_n_chan, CV_32FC1);
        cv::Mat bgr_ave(1, img_n_chan, CV_32FC1);
        cv::Mat inv_cov;

        for (int row = 0; row < height; ++row)
        {
            const void* ptr_bg = NULL;
            const void* ptr_ave = NULL;

            if (colorspace == "rgb" || colorspace == "hsv")
            {
                ptr_bg = new_img.ptr<const uchar>(row);
                ptr_ave = avg_img.ptr<const uchar>(row);
            }
            else if (colorspace == "rgchroma")
            {
                ptr_bg = new_img.ptr<const float>(row);
                ptr_ave = avg_img.ptr<const float>(row);
            }

            for (int col = 0; col < width; ++col)
            {
                int pixel = row * width + col;

                for (int ch = 0; ch < img_n_chan; ++ch)
                {
                    if (colorspace == "rgb" || colorspace == "hsv")
                    {
                        bgr_new.at<float>(0, ch) = ((const uchar*) ptr_bg)[img_n_chan*col + ch];
                        bgr_ave.at<float>(0, ch) = ((const uchar*) ptr_ave)[img_n_chan*col + ch];
                    }
                    else if (colorspace == "rgchroma")
                    {
                        bgr_new.at<float>(0, ch) = ((const float*) ptr_bg)[img_n_chan*col + ch];
                        bgr_ave.at<float>(0, ch) = ((const float*) ptr_ave)[img_n_chan*col + ch];
                    }
                }

                inv_cov = cv::Mat(img_n_chan, img_n_chan, CV_32FC1, &cov_mats_inv[pixel*(img_n_chan*img_n_chan)], sizeof(float)*img_n_chan);

                double mah_dist = cv::Mahalanobis(bgr_new, bgr_ave, inv_cov);
                double unnorm_gaussian = exp(-0.5 * mah_dist);
                double partition = 1.0 / (pow(TWO_PI, 1.5) * sqrt(dets[pixel]));
                float p = partition * unnorm_gaussian;

                prob_data[pixel] = p;
            }
        }

        double min, max;
        cv::log(prob_img, prob_img);
        cv::minMaxLoc(prob_img, &min, &max);
        prob_img.convertTo(prob_img, prob_img.type(), 1.0 / (max - min), -min / (max - min));

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
