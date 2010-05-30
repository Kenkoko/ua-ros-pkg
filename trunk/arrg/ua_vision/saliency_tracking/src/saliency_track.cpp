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
#include <boost/format.hpp>

#include <nmpt/BlockTimer.h>
#include <nmpt/FastSaliency.h>

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
    int salscale;
    int tau0;
    int rad0;
    int doeFeat;
    int dobFeat;
    int colorFeat;
    int useParams;
    int estParams;
    int power;

    int saliencyMapWidth;
    int saliencyMapHeight;
    int imwidth;
    int imheight;

    // Parameter constraints
    int maxPowers;
    int maxScales;
    int maxRad;
    int maxTau;
    int maxsalscale;

    bool initialized;

public:
    SaliencyTracker(ros::NodeHandle& nh, const std::string& transport)
    {
        ros::NodeHandle local_nh("~");
        std::string topic = nh.resolveName("image");

        window_name = topic;
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, .33, .33);

        initialized = false;
        saltracker = NULL;

        // Parameter constraints
        maxPowers = 19;
        maxScales = 6;
        maxRad = 3;
        maxTau = 10;
        maxsalscale = 9;

        // FastSUN saliency tracker parameters
        local_nh.param("spatial_scales", sp_scales, 5);
        local_nh.param("temporal_scales", tm_scales, 0);
        local_nh.param("spatial_size", rad0, 0);
        local_nh.param("temporal_falloff", tau0, 0);
        local_nh.param("image_scale", salscale, 4);
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

        if (salscale > maxsalscale) { salscale = maxsalscale; }
        else if (salscale < 1) { salscale = 1; }

        if (power > maxPowers) { power = maxPowers; }
        else if (power < 0) { power = 0; }

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

        config = newconfig;                // update parameter values

        double mypower = 0.1 + newconfig.distribution_power / 10.0;
        saltracker->setGGDistributionPower(mypower);
        saltracker->setUseDoEFeatures(newconfig.use_temporal_features);
        saltracker->setUseDoBFeatures(newconfig.use_spatial_features);
        saltracker->setUseColorInformation(newconfig.use_color_contrast);
        saltracker->setUseGGDistributionParams(newconfig.use_histogram);
        saltracker->setEstimateGGDistributionParams(newconfig.estimate_histogram);

        ROS_INFO_STREAM("Setting Current FastSaliency Parameters.");
        ROS_INFO_STREAM("  Generalized Gaussian Distribution Power: " << newconfig.distribution_power);
        ROS_INFO_STREAM("  Use Difference of Exponential Features:  " << newconfig.use_temporal_features);
        ROS_INFO_STREAM("  Use Difference of Box Features:          " << newconfig.use_spatial_features);
        ROS_INFO_STREAM("  Use Color Information:                   " << newconfig.use_color_contrast);
        ROS_INFO_STREAM("  Use Estimated GG Distribution Params:    " << newconfig.use_histogram);
        ROS_INFO_STREAM("  Estimate GG Dist Params from Data:       " << newconfig.estimate_histogram);
    }

    void initialize(IplImage* current_frame)
    {
        ROS_INFO("Initializing saliency tracker");

        imwidth = current_frame->width;
        imheight = current_frame->height;

        saliencyMapWidth = imwidth / salscale;
        saliencyMapHeight = imheight / salscale;

        ROS_INFO("ih:%d, iw:%d; h: %d, w: %d\n", imwidth, imheight, saliencyMapWidth, saliencyMapHeight);

        int nspatial = sp_scales + 1;       // [2 3 4 5 ...]
        int ntemporal = tm_scales + 2;      // [2 3 4 5 ...]
        float first_tau = 1.0 / (tau0 + 1); // [1 1/2 1/3 1/4 1/5 ...]
        int first_rad = (1 << rad0) / 2;    // [0 1 2 4 8 16 ...]

        saltracker = new FastSaliency(saliencyMapWidth, saliencyMapHeight, ntemporal, nspatial, first_tau, first_rad);

        dynamic_reconfigure::Server<Config>::CallbackType f = boost::bind(&SaliencyTracker::reconfig, this, _1, _2);
        srv.setCallback(f);

        //Make some intermediate image representations:

        //(1) The downsized representation of the original image frame
        small_color_image = cvCreateImage(cvSize(saliencyMapWidth, saliencyMapHeight),
                                          current_frame->depth, current_frame->nChannels);

        //(2) The floating point image that is passed to the saliency algorithm; this also
        //doubles as an intermediate representation for the output.
        small_float_image = cvCreateImage(cvSize(saliencyMapWidth, saliencyMapHeight),
                                          IPL_DEPTH_32F, current_frame->nChannels);

        //(3) An image to visualize both the original image and the saliency image simultaneously
        composite_image = cvCreateImage(cvSize(saliencyMapWidth * 2, saliencyMapHeight),
                                        current_frame->depth, current_frame->nChannels);

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

