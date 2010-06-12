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

#include <cv_bridge/CvBridge.h>
#include <background_filters/GetBgStats.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <nmpt/OpenCVBoxFilter.h>
#include <background_filters/common.h>

#define TWO_PI 6.28318531

using namespace std;

class BackgroundSubtractor
{
private:
    cv::Mat avg_img;

    vector<float> cov_mats_inv;
    vector<float> dets;
    vector<double> partition;

    int img_n_chan;
    string colorspace;

    image_transport::Subscriber image_sub;
    ros::Publisher prob_img_pub;

public:
    BackgroundSubtractor(ros::NodeHandle& nh, const std::string& transport)
    {
        ros::ServiceClient client = nh.serviceClient<background_filters::GetBgStats>("get_background_stats");

        //cvStartWindowThread();
        background_filters::GetBgStats srv;
        sensor_msgs::CvBridge bridge;

        //cvNamedWindow("prob_img");

        if (client.call(srv))
        {
            bridge.fromImage(srv.response.average_background);
            avg_img = cv::Mat(bridge.toIpl()).clone();
            colorspace = srv.response.colorspace;
            cov_mats_inv = srv.response.covariance_matrix_inv;
            dets = srv.response.covariance_matrix_dets;

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

        std::string topic = nh.resolveName("image");
        image_transport::ImageTransport it(nh);

        image_sub = it.subscribe(topic, 1, &BackgroundSubtractor::handle_image, this);
        prob_img_pub = nh.advertise<sensor_msgs::Image>("probability_image", 1);
    }

    ~BackgroundSubtractor()
    {
        cvDestroyWindow("prob_img");
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
            difference<const uchar>(new_img, msg_ptr->header.stamp);
        }
        else if (colorspace == "rgchroma")
        {
            difference<const float>(new_img, msg_ptr->header.stamp);
        }
    }

    template <class T>
    void difference(const cv::Mat& new_img, ros::Time stamp)
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

        IplImage prob_img_ipl = prob_img;
        sensor_msgs::Image::Ptr prob_msg = sensor_msgs::CvBridge::cvToImgMsg(&prob_img_ipl);
        prob_msg->header.stamp = stamp;
        prob_img_pub.publish(prob_msg);

        cv::Mat result = prob_img.clone();
        cv::normalize(prob_img, prob_img, 0.0, 1.0, cv::NORM_MINMAX);
        //cv::imshow("prob_img", prob_img);
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "background_subtractor", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    if (n.resolveName("image") == "/image")
    {
        ROS_WARN("background_subtractor: image has not been remapped! Typical command-line usage:\n"
        "\t$ ./background_subtractor image:=<image topic> [transport]");
    }

    BackgroundSubtractor subtractor(n, (argc > 1) ? argv[1] : "raw");

    ros::spin();
    return 0;
}
