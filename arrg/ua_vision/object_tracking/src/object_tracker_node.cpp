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

#include <cmath>

#include <ros/ros.h>

#include <cv_bridge/CvBridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>

#include <background_filters/GetBgStats.h>
#include <background_filters/common.h>
#include <background_filters/background_subtractor.h>
#include <background_filters/lbp_background_subtractor.h>

#include <object_tracking/object_tracker.h>

class ObjectTrackerNode
{
private:
    cv::Mat mlr_weights;

    BackgroundSubtractor bg_sub;
    LBPModel lbp_model;
    ObjectTracker object_tracker;

    bool go;
    bool lbp_init;
    int counter;

    ros::Publisher marker_pub;
    ros::Publisher object_pub;
    image_transport::Subscriber image_sub;
    ros::Subscriber info_sub;
    image_geometry::PinholeCameraModel cam_model;
    tf::TransformListener tf_listener;

public:
    ObjectTrackerNode(ros::NodeHandle& nh, const std::string& transport)
    {
        ros::ServiceClient client = nh.serviceClient<background_filters::GetBgStats>("get_background_stats");
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        object_pub = nh.advertise<geometry_msgs::PoseArray>("overhead_objects", 1);

        background_filters::GetBgStats srv;
        sensor_msgs::CvBridge bridge;

        go = false;
        lbp_init = false;
        counter = 0;

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
        std::string topic_info = nh.resolveName("camera_info");
        image_transport::ImageTransport it(nh);

        image_sub = it.subscribe(topic, 1, &ObjectTrackerNode::handle_image, this);
        info_sub = nh.subscribe<sensor_msgs::CameraInfo>(topic_info, 1, &ObjectTrackerNode::handle_info, this);
    }

    void handle_info(const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        cam_model.fromCameraInfo(info_msg);
    }

    void handle_image(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        sensor_msgs::CvBridge bridge;
        cv::Mat original;

        try
        {
            if (bg_sub.colorspace == "rgb")
            {
                original = cv::Mat(bg_sub.avg_img.size(), CV_8UC3);
                cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), original, bg_sub.avg_img.size());
            }
            else if (bg_sub.colorspace == "hsv")
            {
                original = cv::Mat(bg_sub.avg_img.size(), CV_8UC3);
                cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), original, bg_sub.avg_img.size());
                cv::cvtColor(original, original, CV_BGR2HSV);
            }
            else if (bg_sub.colorspace == "rgchroma")
            {
                cv::Mat temp(bg_sub.avg_img.size(), CV_8UC3);
                cv::resize(cv::Mat(bridge.imgMsgToCv(msg_ptr, "bgr8")), temp, bg_sub.avg_img.size());
                original = cv::Mat(bg_sub.avg_img.size(), CV_32FC2);
                convertToChroma(temp, original);
            }
        }
        catch (sensor_msgs::CvBridgeException error)
        {
            ROS_ERROR("CvBridgeError");
        }

        cv::Mat neg_log_lik_img = bg_sub.subtract_background(original);

        if (!lbp_init)
        {
            cv::Mat temp = original.clone();
            lbp_model.initialize(temp);
            lbp_init = true;
        }

        if (lbp_model.do_updates && counter++ > 150) { lbp_model.do_updates = false; ROS_INFO("LBP Ready."); }

        cv::Mat temp = original.clone();
        lbp_model.update(temp);

        cv::Mat lbp_foreground_img = lbp_model.foreground;

        if (!go)
        {
            cv::namedWindow("blah");
            char k = cv::waitKey(1);
            if( k == 27 || k == 's' || k == 'S' ) { go = true; cv::destroyWindow("blah"); };
        }

        if (go)
        {
            object_tracker.find_objects(neg_log_lik_img, lbp_foreground_img, original, msg_ptr->header.stamp);

            if (cam_model.width() == 0) { ROS_WARN("No CameraInfo message received yet or uncalibrated camera"); return; }

            geometry_msgs::PoseArray obj_pose_array;
            obj_pose_array.header.stamp = msg_ptr->header.stamp;
            obj_pose_array.header.frame_id = "/map";

            for (size_t i = 0; i < object_tracker.objects.size(); ++i)
            {
                cv::Point2d point_img;
                double scale = msg_ptr->width / (double) bg_sub.avg_img.cols;

                const cv::RotatedRect& obj_rect = object_tracker.objects[i].tight_bounding_box;
                point_img.x = obj_rect.center.x * scale;
                point_img.y = obj_rect.center.y * scale;

                cv::Point3d point_map;
                cam_model.projectPixelTo3dRay(point_img, point_map);

                tf::StampedTransform transform;
                try
                {
                    ros::Time acquisition_time = ros::Time(0);
                    ros::Duration timeout(1.0 / 30);
                    //tf_listener.waitForTransform("/map", cam_model.tfFrame(), acquisition_time, timeout);
                    tf_listener.lookupTransform("/map", cam_model.tfFrame(), acquisition_time, transform);
                }
                catch (tf::TransformException& ex)
                {
                    ROS_WARN("[object_tracker] TF exception:\n%s", ex.what());
                    continue;
                }

                tf::Point tfp = transform.getOrigin();
                //ROS_INFO("Transform = (%f, %f, %f)", tfp.getX(), tfp.getY(), tfp.getZ());

                point_map *= tfp.getZ();
                //ROS_INFO("Point in image frame = (%f, %f, %f)", point_map.x, point_map.y, point_map.z);

                btQuaternion quat;
                quat.setRPY(0.0f, 0.0f, obj_rect.angle * M_PI / 180.0 - M_PI / 2.0);

                double width = 0.04;
                visualization_msgs::Marker marker;
                marker.header.frame_id = "/map";
                marker.header.stamp = msg_ptr->header.stamp;
                marker.ns = "overhead_camera_objects";
                marker.id = object_tracker.objects[i].id;
                marker.type = visualization_msgs::Marker::CUBE;;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = tfp.x() - point_map.x;
                marker.pose.position.y = tfp.y() + point_map.y;
                marker.pose.position.z = width / 2;
                marker.pose.orientation.x = quat.x();
                marker.pose.orientation.y = quat.y();
                marker.pose.orientation.z = quat.z();
                marker.pose.orientation.w = quat.w();
                marker.scale.x = obj_rect.size.width * scale / cam_model.fx() * tfp.getZ();
                marker.scale.y = obj_rect.size.height * scale / cam_model.fy() * tfp.getZ();
                marker.scale.z = width;
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = 0.8;
                marker.lifetime = ros::Duration(1.0);
                marker_pub.publish(marker);

                obj_pose_array.poses.push_back(marker.pose);

                //ROS_INFO("Point in map frame = (%f, %f, %f)", marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
                //ROS_INFO("Object size (%f, %f) and rotation is %f degrees", marker.scale.x, marker.scale.y, obj_rects[i].angle);
            }

            object_pub.publish(obj_pose_array);
        }
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "object_tracker", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    if (n.resolveName("image") == "/image")
    {
        ROS_WARN("object_tracker: image has not been remapped! Typical command-line usage:\n"
                 "\t$ ./background_subtractor image:=<image topic> [transport]");
    }

    ObjectTrackerNode tracker_node(n, (argc > 1) ? argv[1] : "raw");

    ros::spin();
    return 0;
}
