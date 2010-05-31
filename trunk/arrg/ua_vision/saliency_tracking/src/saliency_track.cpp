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

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point.h>

#include <dynamic_reconfigure/server.h>
#include <driver_base/SensorLevels.h>
#include <saliency_tracking/FastSaliencyConfig.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>

#include <nmpt/BlockTimer.h>
#include <nmpt/FastSaliency.h>

typedef driver_base::SensorLevels Levels;

class SaliencyTracker
{
private:
    image_transport::Subscriber sub;
    sensor_msgs::CvBridge cv_bridge;
    ros::Publisher saliency_poi_pub;

    /** dynamic parameter configuration */
    typedef saliency_tracking::FastSaliencyConfig Config;
    Config config;
    dynamic_reconfigure::Server<Config> srv;
    boost::mutex st_mutex;

    std::string window_name;
    CvFont font;

    IplImage* small_color_image;
    IplImage* small_float_image;
    IplImage* composite_image;

    //The timer is used to track the frame rate
    BlockTimer timer;
    FastSaliency* saltracker;

    // FastSUN saliency tracker parameters
    int salscale;

    int saliencyMapWidth;
    int saliencyMapHeight;
    int imwidth;
    int imheight;

    bool initialized;

public:
    SaliencyTracker(ros::NodeHandle& nh, const std::string& transport)
    {
        ros::NodeHandle local_nh("~");
        std::string topic = nh.resolveName("image");

        window_name = topic;
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, .33, .33);

        initialized = false;
        small_color_image = NULL;
        small_float_image = NULL;
        composite_image = NULL;
        saltracker = NULL;

        // FastSUN saliency tracker parameters
        local_nh.param("img_scale", config.img_scale, 4);
        local_nh.param("spatial_scales", config.spatial_scales, 5);
        local_nh.param("temporal_scales", config.temporal_scales, 0);
        local_nh.param("spatial_size", config.spatial_size, 0);
        local_nh.param("temporal_falloff", config.temporal_falloff, 0);
        local_nh.param("distribution_power", config.distribution_power, 9);
        local_nh.param("use_spatial_features", config.use_spatial_features, true);
        local_nh.param("use_temporal_features", config.use_temporal_features, true);
        local_nh.param("use_color_contrast", config.use_color_contrast, true);
        local_nh.param("estimate_histogram", config.estimate_histogram, true);
        local_nh.param("use_histogram", config.use_histogram, true);

        cvStartWindowThread();

        image_transport::ImageTransport it(nh);
        sub = it.subscribe(topic, 1, &SaliencyTracker::image_cb, this, transport);

        saliency_poi_pub = nh.advertise<geometry_msgs::Point>("saliency_poi", 1);
    }

    ~SaliencyTracker()
    {
        cvDestroyWindow(window_name.c_str());
    }

    void reconfig(Config &newconfig, uint32_t level)
    {
        ROS_INFO("dynamic reconfigure level 0x%x", level);

        if (initialized) { config = newconfig; }                // update parameter values

        boost::mutex::scoped_lock lock(st_mutex);

        if (level & Levels::RECONFIGURE_CLOSE)
        {
            saliencyMapWidth = imwidth / config.img_scale;
            saliencyMapHeight = imheight / config.img_scale;

            //Make some intermediate image representations:
            if (small_color_image != NULL)
            {
                cvReleaseImage(&small_color_image);
                small_color_image = NULL;
            }

            if (small_float_image != NULL)
            {
                cvReleaseImage(&small_float_image);
                small_float_image = NULL;
            }

            if (composite_image != NULL)
            {
                cvReleaseImage(&composite_image);
                composite_image = NULL;
            }

            //(1) The downsized representation of the original image frame
            small_color_image = cvCreateImage(cvSize(saliencyMapWidth, saliencyMapHeight), IPL_DEPTH_8U, 3);

            //(2) The floating point image that is passed to the saliency algorithm; this also
            //doubles as an intermediate representation for the output.
            small_float_image = cvCreateImage(cvSize(saliencyMapWidth, saliencyMapHeight), IPL_DEPTH_32F, 3);

            //(3) An image to visualize both the original image and the saliency image simultaneously
            composite_image = cvCreateImage(cvSize(saliencyMapWidth * 2, saliencyMapHeight), IPL_DEPTH_8U, 3);

            int nspatial = config.spatial_scales + 1;               // [2 3 4 5 ...]
            int ntemporal = config.temporal_scales + 2;             // [2 3 4 5 ...]
            float first_tau = 1.0 / (config.temporal_falloff + 1);  // [1 1/2 1/3 1/4 1/5 ...]
            int first_rad = (1 << config.spatial_size) / 2;         // [0 1 2 4 8 16 ...]

            if (saltracker != NULL)
            {
                delete(saltracker);
                saltracker = NULL;
            }

            saltracker = new FastSaliency(saliencyMapWidth, saliencyMapHeight, ntemporal, nspatial, first_tau, first_rad);

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
    }

    void initialize(IplImage* current_frame)
    {
        ROS_INFO("Initializing saliency tracker");

        imwidth = current_frame->width;
        imheight = current_frame->height;

        dynamic_reconfigure::Server<Config>::CallbackType f = boost::bind(&SaliencyTracker::reconfig, this, _1, _2);
        srv.setCallback(f);

        cvNamedWindow(window_name.c_str(), CV_WINDOW_AUTOSIZE);
        initialized = true;
        timer.blockStart(0);
    }

    void image_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        if (cv_bridge.fromImage(*msg, "bgr8"))
        {
            IplImage* current_frame = cv_bridge.toIpl();
            if (!initialized) { initialize(current_frame); }

            boost::mutex::scoped_lock lock(st_mutex);

            //Put the current frame into the format expected by the saliency algorithm
            cvResize(current_frame, small_color_image, CV_INTER_LINEAR);
            cvConvert(small_color_image, small_float_image);

            //Call the "updateSaliency" method, and time how long it takes to run
            timer.blockStart(1);
            saltracker->updateSaliency(small_float_image);
            cvNormalize(saltracker->salImageFloat, saltracker->salImageFloat, 0, 1, CV_MINMAX, NULL);
            timer.blockStop(1);

            //Paste the original color image into the left half of the display image
            CvRect half = cvRect(0, 0, saliencyMapWidth, saliencyMapHeight);
            cvSetImageROI(composite_image, half);
            cvCopy(small_color_image, composite_image, NULL);

            cvCvtColor(saltracker->salImageFloat, small_float_image, CV_GRAY2BGR);

            // Find and publish poi
            double min, max;
            CvPoint p_min, p_max;
            cvMinMaxLoc(saltracker->salImageFloat, &min, &max, &p_min, &p_max);

            //printf("intensity=%f at (%d, %d)\n", max, max_x, max_y);
            geometry_msgs::Point poi;
            poi.x = p_max.x * salscale;
            poi.y = p_max.y * salscale;
            poi.z = 0.0;
            saliency_poi_pub.publish(poi);

            // Paste the saliency map into the right half of the color image
            half = cvRect(saliencyMapWidth, 0, saliencyMapWidth, saliencyMapHeight);
            cvSetImageROI(composite_image, half);
            cvConvertScale(small_float_image, composite_image, 255, 0);
            cvResetImageROI(composite_image);

            timer.blockStop(0);

            double tot = timer.getTotTime(0);
            double fps = timer.getTotTime(1);

            timer.blockReset(0);
            timer.blockReset(1);
            timer.blockStart(0);

            //Print the timing information to the display image
            char str[1000] = {'\0'};
            sprintf(str,"FastSUN: %03.1f MS;   Total: %03.1f MS", 1000.0*fps, 1000.0*tot);
            cvPutText( composite_image, str, cvPoint(20,20), &font, CV_RGB(255,0,255) );

            cvShowImage(window_name.c_str(), composite_image);
        }
        else
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
        ROS_WARN("saliency_tracker: image has not been remapped! Typical command-line usage:\n"
                 "\t$ ./saliency_tracker image:=<image topic> [transport]");
    }

    SaliencyTracker tracker(n, (argc > 1) ? argv[1] : "raw");

    ros::spin();
    return 0;
}

