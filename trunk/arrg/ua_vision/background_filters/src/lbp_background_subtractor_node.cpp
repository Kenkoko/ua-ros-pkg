/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Arizona Robotics Research Group.
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

#include <cv_bridge/CvBridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <background_filters/lbp_background_subtractor.h>

int num_model_frames = 150;
int fr = 0;
LBPModel lbp_model;

void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
{
    sensor_msgs::CvBridge bridge;
    cv::Mat tmp_frame;
    cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), tmp_frame, cv::Size(640, 480));

    if (fr++ == 1)
    {
        lbp_model.initialize( tmp_frame );
        return;
    }

    double t = (double)cvGetTickCount();

    cv::Mat resized_frame, segmented, mask, threshMask;

    if (fr > num_model_frames) { lbp_model.do_updates = false; }
    lbp_model.update( tmp_frame );

    t = (double)cvGetTickCount() - t;
    printf( "%d. %.1fms\n", fr, t/(cvGetTickFrequency()*1000.) );

    cv::resize(tmp_frame, resized_frame, cv::Size( lbp_model.foreground.cols, lbp_model.foreground.rows ));
    cv::GaussianBlur(lbp_model.foreground, mask, cv::Size(5,5), .95, .95);
    cv::threshold(mask, threshMask, 160, 255, cv::THRESH_BINARY);
    resized_frame.copyTo( segmented, threshMask );

    cv::imshow("Segmented", segmented);
    cv::imshow("LBP Subtraction", lbp_model.foreground);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "lbp_bg_subtractor", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    if (nh.resolveName("image") == "/image")
    {
        ROS_WARN("object_tracker: image has not been remapped! Typical command-line usage:\n"
        "\t$ ./background_subtractor image:=<image topic> [transport]");
    }

    std::string topic = nh.resolveName("image");
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe(topic, 1, &handle_image);

    cvNamedWindow("Segmented", 1);
    cvMoveWindow("Segmented", 0, 0);
    cvNamedWindow("LBP Subtraction", 3);
    cvMoveWindow("LBP Subtraction", 0, 400);

    cv::startWindowThread();

    ros::spin();
    return 0;
}
