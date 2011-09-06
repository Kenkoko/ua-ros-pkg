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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <background_filters/common.h>
#include <background_filters/background_subtractor.h>

using namespace std;

class BackgroundSubtractorNode
{
private:
    BackgroundSubtractor bg_sub;
    image_transport::Subscriber image_sub;
    ros::Publisher prob_img_pub;

public:
    BackgroundSubtractorNode(ros::NodeHandle& nh, const std::string& transport)
    {
        ros::ServiceClient client = nh.serviceClient<background_filters::GetBgStats>("get_background_stats");

        background_filters::GetBgStats srv;
        sensor_msgs::CvBridge bridge;

        if (client.call(srv))
        {
            bridge.fromImage(srv.response.average_background);

            bg_sub.initialize(srv.response.colorspace,
                              cv::Mat(bridge.toIpl()).clone(),
                              srv.response.covariance_matrix_inv,
                              srv.response.covariance_matrix_dets);
        }
        else
        {
            ROS_ERROR("Failed to call service get_bg_stats");
        }

        std::string topic = nh.resolveName("image");
        image_transport::ImageTransport it(nh);

        image_sub = it.subscribe(topic, 1, &BackgroundSubtractorNode::handle_image, this);
        prob_img_pub = nh.advertise<sensor_msgs::Image>("probability_image", 1);
    }

    void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        sensor_msgs::CvBridge bridge;
        cv::Mat new_img;

        try
        {
            if (bg_sub.colorspace == "rgb")
            {
                new_img = cv::Mat(bg_sub.avg_img.size(), CV_8UC3);
                cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), new_img, bg_sub.avg_img.size());
            }
            else if (bg_sub.colorspace == "hsv")
            {
                new_img = cv::Mat(bg_sub.avg_img.size(), CV_8UC3);
                cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), new_img, bg_sub.avg_img.size());
                cv::cvtColor(new_img, new_img, CV_BGR2HSV);
            }
            else if (bg_sub.colorspace == "rgchroma")
            {
                cv::Mat temp(bg_sub.avg_img.size(), CV_8UC3);
                cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), temp, bg_sub.avg_img.size());
                new_img = cv::Mat(bg_sub.avg_img.size(), CV_32FC2);
                convertToChroma(temp, new_img);
            }
        }
        catch (sensor_msgs::CvBridgeException error)
        {
            ROS_ERROR("CvBridgeError");
        }

        cv::Mat prob_img(new_img.size(), CV_32FC1);
        bg_sub.subtract_background(new_img, prob_img);
        IplImage prob_img_ipl = prob_img;
        sensor_msgs::Image::Ptr prob_msg = sensor_msgs::CvBridge::cvToImgMsg(&prob_img_ipl);
        prob_msg->header.stamp = msg_ptr->header.stamp;
        prob_img_pub.publish(prob_msg);
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

    BackgroundSubtractorNode subtractor_node(n, (argc > 1) ? argv[1] : "raw");

    ros::spin();
    return 0;
}
