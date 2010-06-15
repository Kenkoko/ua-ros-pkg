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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/CvBridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/bind.hpp>
#include <nmpt/OpenCVBoxFilter.h>
#include <background_filters/common.h>


using namespace std;

class ObjectTracker
{
private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::Image> camera_sub;
    message_filters::Subscriber<sensor_msgs::Image> saliency_sub;
    message_filters::Subscriber<sensor_msgs::Image> fg_objects_sub;
    message_filters::Synchronizer<SyncPolicy>* sync;

    sensor_msgs::CvBridge camera_bridge;
    sensor_msgs::CvBridge saliency_bridge;
    sensor_msgs::CvBridge fg_objects_bridge;

public:
    ObjectTracker(ros::NodeHandle& nh)
    {
        camera_sub.subscribe(nh, "image", 1);
        saliency_sub.subscribe(nh, "saliency_img", 1);
        fg_objects_sub.subscribe(nh, "probability_image", 1);

        // ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(10)
        sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), camera_sub, fg_objects_sub, saliency_sub);
        sync->registerCallback(boost::bind(&ObjectTracker::process_images, this, _1, _2, _3));
    }

    ~ObjectTracker()
    {
        delete sync;
    }

    void process_images(const sensor_msgs::ImageConstPtr& camera_msg, const sensor_msgs::ImageConstPtr& fg_objects_msg, const sensor_msgs::ImageConstPtr& saliency_msg)
    {
        ROS_INFO_STREAM("Raw image time:          " << camera_msg->header.stamp);
        ROS_INFO_STREAM("Foreground objects time: " << fg_objects_msg->header.stamp);
        ROS_INFO_STREAM("Saliency time:           " << saliency_msg->header.stamp);

        cv::Mat camera_img(camera_bridge.imgMsgToCv(camera_msg));
        cv::Mat fg_objects_img(fg_objects_bridge.imgMsgToCv(fg_objects_msg));
        cv::Mat saliency_img(saliency_bridge.imgMsgToCv(saliency_msg));

        double w = -1/5.0, b = 4.0;
        cv::Mat fg_prob_img = fg_objects_img;
        fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), w, b);
        cv::exp(fg_prob_img, fg_prob_img);
        fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), 1, 1);
        cv::divide(1.0, fg_prob_img, fg_prob_img);

        cv::namedWindow("fg_prob_img");
        cv::imshow("fg_prob_img", fg_prob_img);

        double sum = cv::sum(fg_prob_img)[0];
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "object_tracker");
    ros::NodeHandle n;

    ObjectTracker tracker(n);
    ros::spin();

    return 0;
}
