/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Arizona Robotics Research Group,
*                      University of Arizona
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

// Author Antons Rebguns with code adapted from image_view and FastSUN

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

#include <driver_base/SensorLevels.h>
#include <dynamic_reconfigure/server.h>
#include <saliency_tracking/FastSaliencyConfig.h>

#include <boost/thread/mutex.hpp>

#include <nmpt/BlockTimer.h>
#include <nmpt/FastSalience.h>

typedef driver_base::SensorLevels Levels;

class SaliencyTracker
{
private:
    image_transport::Subscriber sub;
    ros::Publisher saliency_poi_pub;
    ros::Publisher saliency_img_pub;

    /** dynamic parameter configuration */
    typedef saliency_tracking::FastSaliencyConfig Config;
    dynamic_reconfigure::Server<Config> srv;
    boost::mutex st_mutex;
    Config current_config;

    std::string window_name;

    //The timer is used to track the frame rate
    BlockTimer timer;
    FastSalience* saltracker;

    // FastSUN saliency tracker parameters
    int saliencyMapWidth;
    int saliencyMapHeight;
    int imwidth;
    int imheight;

    bool window_exists;
    bool initialized;

public:
    SaliencyTracker(ros::NodeHandle& nh, const std::string& transport)
    {
        ros::NodeHandle local_nh("~");
        std::string topic = nh.resolveName("image");

        window_name = topic;

        window_exists = false;
        initialized = false;
        saltracker = NULL;

        cvStartWindowThread();

        image_transport::ImageTransport it(nh);
        sub = it.subscribe(topic, 1, &SaliencyTracker::image_cb, this, transport);

        saliency_poi_pub = nh.advertise<geometry_msgs::Point>("saliency_poi", 1);
        saliency_img_pub = nh.advertise<sensor_msgs::Image>("saliency_img", 1);
    }

    ~SaliencyTracker()
    {
        cvDestroyWindow(window_name.c_str());
        delete saltracker;
    }

    void reconfig(Config &config, uint32_t level)
    {
        ROS_INFO("dynamic reconfigure level 0x%x", level);
        boost::mutex::scoped_lock lock(st_mutex);

        if (level & Levels::RECONFIGURE_CLOSE)
        {
            saliencyMapWidth = imwidth / config.img_scale;
            saliencyMapHeight = imheight / config.img_scale;

            //Make some intermediate image representations:


            int nspatial = config.spatial_scales + 1;               // [2 3 4 5 ...]
            int ntemporal = config.temporal_scales + 2;             // [2 3 4 5 ...]
            float first_tau = 1.0 / (config.temporal_falloff + 1);  // [1 1/2 1/3 1/4 1/5 ...]
            int first_rad = (1 << config.spatial_size) / 2;         // [0 1 2 4 8 16 ...]

            if (saltracker != NULL)
            {
                delete(saltracker);
                saltracker = NULL;
            }

            //saltracker = new FastSalience(saliencyMapWidth, saliencyMapHeight, ntemporal, nspatial, first_tau, first_rad);
	    saltracker = new FastSalience(ntemporal, nspatial, first_tau, first_rad);

            ROS_INFO_STREAM("Created new FastSaliency object with the following parameters.");
            ROS_INFO_STREAM("  Width:                                   " << saliencyMapWidth);
            ROS_INFO_STREAM("  Height:                                  " << saliencyMapHeight);
            ROS_INFO_STREAM("  Image Scale:                             " << config.img_scale);
            ROS_INFO_STREAM("  # Temporal Scales:                       " << config.temporal_scales);
            ROS_INFO_STREAM("  # Spatial Scales:                        " << config.spatial_scales);
            ROS_INFO_STREAM("  Slowest Temporal Feature Decay:          " << config.temporal_falloff);
            ROS_INFO_STREAM("  Smallest Spatial Scale Radius:           " << config.spatial_size);
        }

        double mypower = 0.1 + config.distribution_power / 10.0;
        saltracker->setGGDistributionPower(mypower);
        saltracker->setUseDoEFeatures(config.use_temporal_features);
        saltracker->setUseDoBFeatures(config.use_spatial_features);
        saltracker->setUseColorInformation(config.use_color_contrast);
        saltracker->setUseGGDistributionParams(config.use_histogram);
        saltracker->setEstimateGGDistributionParams(config.estimate_histogram);

        ROS_INFO_STREAM("Setting Current FastSaliency Parameters.");
        ROS_INFO_STREAM("  Generalized Gaussian Distribution Power: " << config.distribution_power);
        ROS_INFO_STREAM("  Use Difference of Exponential Features:  " << config.use_temporal_features);
        ROS_INFO_STREAM("  Use Difference of Box Features:          " << config.use_spatial_features);
        ROS_INFO_STREAM("  Use Color Information:                   " << config.use_color_contrast);
        ROS_INFO_STREAM("  Use Estimated GG Distribution Params:    " << config.use_histogram);
        ROS_INFO_STREAM("  Estimate GG Dist Params from Data:       " << config.estimate_histogram);

        current_config = config;
    }

    void initialize(cv::Mat current_frame)
    {
        ROS_INFO("Initializing saliency tracker");

        imwidth = current_frame.cols;
        imheight = current_frame.rows;

        dynamic_reconfigure::Server<Config>::CallbackType f = boost::bind(&SaliencyTracker::reconfig, this, _1, _2);
        srv.setCallback(f);

        initialized = true;
        timer.blockStart(0);
    }

    void image_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
	  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	    cv::Mat current_frame = cv_ptr->image;
            if (!initialized) { initialize(current_frame); }

            boost::mutex::scoped_lock lock(st_mutex);

            //Put the current frame into the format expected by the saliency algorithm
            double ratio = saliencyMapWidth * 1.0 / current_frame.cols;
	    cv::Mat small_float_image;

	    cv::resize(current_frame, small_float_image, cv::Size(0,0), ratio, ratio, cv::INTER_LINEAR);

            //Call the "updateSaliency" method, and time how long it takes to run
            timer.blockStart(1);
            saltracker->updateSalience(small_float_image);
            timer.blockStop(1);

            // Find and publish poi
	    cv::Point p_max;
	    cv::Mat sal_float_image;
	    saltracker->getSalImage(sal_float_image);
	    cv::minMaxLoc(sal_float_image, NULL, NULL, NULL, &p_max);

            geometry_msgs::Point poi;
            poi.x = p_max.x * current_config.img_scale;
            poi.y = p_max.y * current_config.img_scale;
            poi.z = 0.0;

            timer.blockStop(0);

            double total_time = timer.getTotTime(0);
            double fastsun_time = timer.getTotTime(1);

            timer.blockReset(0);
            timer.blockReset(1);
            timer.blockStart(0);

	    // Publish the image.
	    IplImage out_msg = sal_float_image;
	    sensor_msgs::CvBridge bridge_;
	    sensor_msgs::Image::Ptr saliency_msg = bridge_.cvToImgMsg(&out_msg, "passthrough");
            saliency_msg->header.stamp = msg->header.stamp;

            saliency_poi_pub.publish(poi);
            saliency_img_pub.publish(saliency_msg);

            if (current_config.display_img)
            {
                //Print the timing information to the display image
                char str[1000] = {'\0'};
                sprintf(str,"s: %03.1f ms; t: %03.1f ms", 1000.0 * fastsun_time, 1000.0 * total_time);
		cv::putText(sal_float_image, str, cv::Point(5, 10), CV_FONT_HERSHEY_SIMPLEX, 0.33, cv::Scalar(255, 0, 255), 1.0);

                if (!window_exists)
                {
		    cv::namedWindow(window_name.c_str(), CV_WINDOW_AUTOSIZE);
                    window_exists = true;
                }

		cv::imshow(window_name.c_str(), sal_float_image);
            }
            else
            {
                if (window_exists)
                {
                    cvDestroyWindow(window_name.c_str());
                    window_exists = false;
                }
            }
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "saliency_tracker", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    if (n.resolveName("image") == "/image")
    {
        ROS_WARN("saliency_track: image has not been remapped! Typical command-line usage:\n"
                 "\t$ ./saliency_track image:=<image topic> [transport]");
    }

    SaliencyTracker tracker(n, (argc > 1) ? argv[1] : "raw");

    ros::spin();
    return 0;
}
