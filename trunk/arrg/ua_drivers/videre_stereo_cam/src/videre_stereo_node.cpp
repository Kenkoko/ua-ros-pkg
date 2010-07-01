/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Antons Rebguns
*  Copyright (c) 2008, Willow Garage, Inc.
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

/**

@mainpage

@htmlinclude manifest.html

Stereodcam is a driver primarily for communicating with the Videre stereocameras.

<hr>

@section behavior Behavior

The Stereodcam node outputs a "raw_stereo" message, defined in the
"sensor_msgs" package.  This message may either contain a left and
right image, or, in the event of STOC processing, will contain a left
image and disparity image.  It additionally contains the relevant
intrinsic and extrinsic parameters for computing stereo.

<hr>

@section names Names

The default name for the node is "stereodcam", however, this private
namespace is not actually used internally to the node.  This node
primarily makes use of the "stereo" namespace, which it shares with
the "stereoproc" node.  This namespace is both where it looks for
parameters and publishes topics.

The "stereo" name can be remapped through standard topic remapping in
the event that two cameras are sharing the same ROS system.

<hr>

@par Example

Running the driver with non STOC devices
@verbatim
$ roslaunch videre_stereo_cam videre_non_stoc.launch
@endverbatim

Running the driver with STOC devices
@verbatim
$ roslaunch videre_stereo_cam videre.launch
@endverbatim

<hr>

@section topics Topics

Subscribes to (name/type):
- @b "stereo/check_params" : std_msgs/Empty : signal to recheck all of the stereo parameters

The driver publishes different set of topics dependeing the "videre_mode" parameter

Publishes to (name : type : description):
- @b "stereo/left/image_raw" : sensor_msgs/Image : raw image from left camera
- @b "stereo/right/image_raw" : sensor_msgs/Image : raw image from right camera
- @b "stereo/left/camera_info" : sensor_msgs/CameraInfo : left camera model
- @b "stereo/right/camera_info" : sensor_msgs/CameraInfo : right camera model

<hr>

@section parameters Parameters

The camera will set the following parameters after running:
- @b stereo/exposure_max (int) : maximum value for exposure
- @b stereo/exposure_min (int) : maximum value for exposure
- @b stereo/brightness_max (int) : maximum value for brightness
- @b stereo/brightness_min (int) : maximum value for brightness
- @b stereo/gain_max (int) : maximum value for gain
- @b stereo/gain_min (int) : maximum value for gain

The camera will read from the following parameters:
- @b stereo/videre_mode (string) : The processing type that a Videre
  STOC will use.  Value can be "none", "rectified", "disparity" or
  "disparity_raw"
- @b stereo/fps         (double) : Target fps of the camera
- @b stereo/speed       (string) : Firewire isospeed: S100, S200, or S400
- @b stereo/exposure (int)
- @b stereo/gain     (int)
- @b stereo/brightness (int)
- @b stereo/exposure_auto (bool)
- @b stereo/gain_auto  (bool)
- @b stereo/brightness_auto (bool)
- @b stereo/companding (bool)
- @b stereo/hdr (bool)
- @b stereo/texture_threshold (int)
- @b stereo/uniqueness_threshold (int)
- @b stereo/horopter (int)

**/


#include <cstdio>
#include <signal.h>

#include <ros/ros.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_proc/processor.h>
#include <stereo_image_proc/processor.h>

#include <driver_base/SensorLevels.h>
#include <dynamic_reconfigure/server.h>

#include <videre_stereo_cam/VidereStereoCamConfig.h>
#include <videre_stereo_cam/videre_stereo_1394.h>
#include <videre_stereo_cam/stereoimage.h>

typedef driver_base::SensorLevels Levels;

void sigsegv_handler(int sig);

class VidereStereoNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::NodeHandle left_nh_;
    ros::NodeHandle right_nh_;

    sensor_msgs::Image image_;
    sensor_msgs::PointCloud cloud_;
    sensor_msgs::PointCloud2 cloud2_;

    image_transport::CameraPublisher left_camera_pub_;
    image_transport::Publisher left_color_image_pub_;
    image_transport::Publisher left_color_rect_image_pub_;
    image_transport::Publisher left_mono_image_pub_;
    image_transport::Publisher left_mono_rect_image_pub_;

    image_transport::CameraPublisher right_camera_pub_;
    image_transport::Publisher right_color_image_pub_;
    image_transport::Publisher right_color_rect_image_pub_;
    image_transport::Publisher right_mono_image_pub_;
    image_transport::Publisher right_mono_rect_image_pub_;

    ros::Publisher disparity_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher cloud2_pub_;

    image_proc::ImageSet left_set_;
    image_proc::ImageSet right_set_;
    image_proc::Processor processor_;
    stereo_image_proc::StereoProcessor stereo_processor_;

    videre_proc_mode_t videre_mode_;

    // Diagnostic updater stuff
    diagnostic_updater::Updater diagnostic_;
    diagnostic_updater::TimeStampStatus timestamp_diag_;

    int count_;
    double desired_freq_;

    // dynamic parameter configuration
    typedef videre_stereo_cam::VidereStereoCamConfig Config;
    dynamic_reconfigure::Server<Config> srv_;
    Config current_config_;

public:
    dcam::Dcam* stcam_;

    VidereStereoNode() : nh_(ros::NodeHandle()), local_nh_(ros::NodeHandle("~")),
                         left_nh_(ros::NodeHandle("left")), right_nh_(ros::NodeHandle("right")),
                         diagnostic_(), count_(0)
    {
        // Set up segfault handler
        signal(SIGSEGV, &sigsegv_handler);

        // Register a frequency status updater
        diagnostic_.add(timestamp_diag_);
        diagnostic_.add("Frequency Status", this, &VidereStereoNode::freqStatus);

        // Check our guid parameter, or else use first camera
        std::string guid_str;
        local_nh_.param("guid", guid_str, std::string("0"));
        uint64_t guid = strtoll(guid_str.c_str(), NULL, 16);

        stcam_ = new dcam::Dcam(guid);

        diagnostic_.setHardwareID(boost::lexical_cast<std::string>(stcam_->guid));

        // Create the StereoDcam
        ROS_INFO_STREAM("Connecting to camera with GUID " << stcam_->guid << " [" << stcam_->getVendor() << " " << stcam_->getModel() << "]");
        ROS_INFO("Connected camera is%s a STOC device", stcam_->isSTOC ? "" : " NOT");

        // Fetch the camera string and send it to the parameter server if people want it (they shouldn't)
        std::string params(stcam_->getParameters());
        local_nh_.setParam("params", params);
        local_nh_.setParam("exposure_max", (int) stcam_->expMax);
        local_nh_.setParam("exposure_min", (int) stcam_->expMin);
        local_nh_.setParam("gain_max", (int) stcam_->gainMax);
        local_nh_.setParam("gain_min", (int) stcam_->gainMin);
        local_nh_.setParam("brightness_max", (int) stcam_->brightMax);
        local_nh_.setParam("brightness_min", (int) stcam_->brightMin);
        local_nh_.setParam("whitebalance_max", (int) stcam_->whiteBalanceMax);
        local_nh_.setParam("whitebalance_min", (int) stcam_->whiteBalanceMin);

        // Get the ISO speed parameter if available
        std::string str_speed;
        dc1394speed_t speed;
        local_nh_.param("speed", str_speed, std::string("S400"));

        if (str_speed == std::string("S100")) { speed = DC1394_ISO_SPEED_100; }
        else if (str_speed == std::string("S200")) { speed = DC1394_ISO_SPEED_200; }
        else { speed = DC1394_ISO_SPEED_400; }

        // Get the FPS parameter if available;
        double dbl_fps;
        dc1394framerate_t fps;
        local_nh_.param("fps", dbl_fps, 30.0);
        desired_freq_ = dbl_fps;

        if (dbl_fps >= 240.0) { fps = DC1394_FRAMERATE_240; }
        else if (dbl_fps >= 120.0) { fps = DC1394_FRAMERATE_120; }
        else if (dbl_fps >= 60.0) { fps = DC1394_FRAMERATE_60; }
        else if (dbl_fps >= 30.0) { fps = DC1394_FRAMERATE_30; }
        else if (dbl_fps >= 15.0) { fps = DC1394_FRAMERATE_15; }
        else if (dbl_fps >= 7.5) { fps = DC1394_FRAMERATE_7_5; }
        else if (dbl_fps >= 3.75) { fps = DC1394_FRAMERATE_3_75; }
        else { fps = DC1394_FRAMERATE_1_875; }

        // We only support Videre stereo cameras:
        std::string str_mode;
        dc1394video_mode_t mode = VIDERE_STEREO_640x480;
        local_nh_.param("mode", str_mode, std::string("none"));
        if (str_mode == std::string("stereo320x240")) { mode = VIDERE_STEREO_320x240; }
        else if (str_mode == std::string("stereo640x480")) { mode = VIDERE_STEREO_640x480; }

        // Configure camera
        stcam_->setFormat(mode, fps, speed);

        // Dynamic reconfigure
        dynamic_reconfigure::Server<Config>::CallbackType f = boost::bind(&VidereStereoNode::reconfig, this, _1, _2);
        srv_.setCallback(f);

        // Start the camera
        stcam_->start();
        usleep(200000);
    }

    void reconfig(Config &config, uint32_t level)
    {
        ROS_INFO("Camera reconfigure request received, level 0x%x", level);

        std::string tf_prefix = tf::getPrefixParam(local_nh_);
        current_config_.frame_id = tf::resolve(tf_prefix, config.frame_id);

        if (level & driver_base::SensorLevels::RECONFIGURE_CLOSE)
        {
            // Get the videre processing mode if available:
            if (config.videre_mode == std::string("test")) { videre_mode_ = PROC_MODE_TEST; }
            else if (config.videre_mode == std::string("rectified")) { videre_mode_ = PROC_MODE_RECTIFIED; }
            else if (config.videre_mode == std::string("disparity_raw")) { videre_mode_ = PROC_MODE_DISPARITY_RAW; }
            else if (config.videre_mode == std::string("disparity")) { videre_mode_ = PROC_MODE_DISPARITY; }
            else { videre_mode_ = PROC_MODE_NONE; }

            // Check if selected "videre mode" is supported by the camera
            if (!stcam_->isSTOC && videre_mode_ != PROC_MODE_NONE)
            {
                ROS_WARN("Videre Modes other than PROC_MODE_NONE are only available on STOC devices");
                ROS_WARN("Setting mode to none");
                videre_mode_ = PROC_MODE_NONE;
                config.videre_mode = "none";
            }
            else
            {
                ROS_INFO("Setting mode to %s", config.videre_mode.c_str());
            }

            config.calculate_points = config.calculate_points && (videre_mode_ == PROC_MODE_DISPARITY || videre_mode_ == PROC_MODE_DISPARITY_RAW);
            config.calculate_points2 = config.calculate_points2 && (videre_mode_ == PROC_MODE_DISPARITY || videre_mode_ == PROC_MODE_DISPARITY_RAW);
            config.convert_to_color = config.convert_to_color && stcam_->isColor;
            current_config_.calculate_points = config.calculate_points;
            current_config_.calculate_points2 = config.calculate_points2;
            current_config_.convert_to_color = config.convert_to_color;

            ROS_INFO("Color conversion from Bayer pattern is %s", config.convert_to_color ? "Enabled" : "Disabled");

            if (stcam_->isSTOC)
            {
                stcam_->setProcMode(videre_mode_);
                ROS_INFO("STOC: Point cloud calculation is %s", config.calculate_points ? "Enabled" : "Disabled");
                ROS_INFO("STOC: Point cloud 2 calculation is %s", config.calculate_points2 ? "Enabled" : "Disabled");
            }

            shutdown_topics();
            advertise_topics();
        }

        stcam_->setExposure(config.exposure, config.exposure_auto);
        stcam_->setGain(config.gain, config.gain_auto);
        stcam_->setBrightness(config.brightness, config.brightness_auto);
        stcam_->setWhiteBalance(config.whitebalance_blue, config.whitebalance_red, config.whitebalance_auto);
        stcam_->setCompanding(config.companding);
        stcam_->setHDR(config.hdr);

        ROS_INFO("Stereo camera's frame ID is %s", current_config_.frame_id.c_str());

        if (config.exposure_auto) { ROS_INFO("Setting Exposure to Auto setting"); }
        else { ROS_INFO("Setting Exposure to %d", config.exposure); }

        if (config.gain_auto) { ROS_INFO("Setting Gain to Auto setting"); }
        else { ROS_INFO("Setting Gain to %d", config.gain); }

        if (config.brightness_auto) { ROS_INFO("Setting Brightness to Auto setting"); }
        else { ROS_INFO("Setting Brightness to %d", config.brightness); }

        if (config.whitebalance_auto) { ROS_INFO("Setting Whitebalance to Auto setting"); }
        else { ROS_INFO("Setting Whitebalance to (B: %d, R: %d)", config.whitebalance_blue, config.whitebalance_red); }

        if (config.companding) { ROS_INFO("Companding mode is Enabled"); }
        else { ROS_INFO("Companding mode is Disabled"); }

        if (config.hdr) { ROS_INFO("High Dynamic Range mode is Enabled"); }
        else { ROS_INFO("High Dynamic Range mode is Disabled"); }

        if (videre_mode_ == PROC_MODE_DISPARITY || videre_mode_ == PROC_MODE_DISPARITY_RAW)
        {
            stcam_->setUniqueThresh(config.uniqueness_threshold);
            stcam_->setTextureThresh(config.texture_threshold);
            stcam_->setSpeckleSize(config.speckle_size);
            stcam_->setSpeckleDiff(config.speckle_range);
            stcam_->setHoropter(0);//config.horopter); // TODO: make horopter stuff work correctly

            ROS_INFO("STOC: uniqueness threshold is set to %d", config.uniqueness_threshold);
            ROS_INFO("STOC: texture threshold is set to %d", config.texture_threshold);
            ROS_INFO("STOC: speckle size is set to %d", config.speckle_size);
            ROS_INFO("STOC: speckle range is set to %d", config.speckle_range);
            ROS_INFO("STOC: horopter is set to %d", config.horopter);
        }

        ROS_INFO("-------------------------------------------");
    }

    void shutdown_topics()
    {
        left_camera_pub_.shutdown();
        left_color_image_pub_.shutdown();
        left_color_rect_image_pub_.shutdown();
        left_mono_image_pub_.shutdown();
        left_mono_rect_image_pub_.shutdown();

        right_camera_pub_.shutdown();
        right_color_image_pub_.shutdown();
        right_color_rect_image_pub_.shutdown();
        right_mono_image_pub_.shutdown();
        right_mono_rect_image_pub_.shutdown();

        disparity_pub_.shutdown();
        cloud_pub_.shutdown();
        cloud2_pub_.shutdown();
    }

    void advertise_topics()
    {
        switch (videre_mode_)
        {
            // publish raw left and right images
            case PROC_MODE_TEST:
            case PROC_MODE_OFF:
            case PROC_MODE_NONE:
                left_camera_pub_ = image_transport::ImageTransport(left_nh_).advertiseCamera("image_raw", 1);
                right_camera_pub_ = image_transport::ImageTransport(right_nh_).advertiseCamera("image_raw", 1);

                if (current_config_.convert_to_color)
                {
                    left_color_image_pub_ = image_transport::ImageTransport(left_nh_).advertise("image_color", 1);
                    left_color_rect_image_pub_ = image_transport::ImageTransport(left_nh_).advertise("image_rect_color", 1);
                    left_mono_image_pub_ = image_transport::ImageTransport(left_nh_).advertise("image_mono", 1);
                    left_mono_rect_image_pub_ = image_transport::ImageTransport(left_nh_).advertise("image_rect", 1);

                    right_color_image_pub_ = image_transport::ImageTransport(right_nh_).advertise("image_color", 1);
                    right_color_rect_image_pub_ = image_transport::ImageTransport(right_nh_).advertise("image_rect_color", 1);
                    right_mono_image_pub_ = image_transport::ImageTransport(right_nh_).advertise("image_mono", 1);
                    right_mono_rect_image_pub_ = image_transport::ImageTransport(right_nh_).advertise("image_rect", 1);
                }

                break;
            // publish monochrome rectified left and right images
            case PROC_MODE_RECTIFIED:
                left_camera_pub_ = image_transport::ImageTransport(left_nh_).advertiseCamera("image_rect", 1);
                right_camera_pub_ = image_transport::ImageTransport(right_nh_).advertiseCamera("image_rect", 1);

                break;
            // publish raw left image and disparity image from the camera
            case PROC_MODE_DISPARITY_RAW:
                left_camera_pub_ = image_transport::ImageTransport(left_nh_).advertiseCamera("image_raw", 1);
                disparity_pub_ = nh_.advertise<stereo_msgs::DisparityImage>("disparity", 1);

                if (current_config_.convert_to_color)
                {
                    left_color_image_pub_ = image_transport::ImageTransport(left_nh_).advertise("image_color", 1);
                    left_color_rect_image_pub_ = image_transport::ImageTransport(left_nh_).advertise("image_rect_color", 1);
                    left_mono_image_pub_ = image_transport::ImageTransport(left_nh_).advertise("image_mono", 1);
                    left_mono_rect_image_pub_ = image_transport::ImageTransport(left_nh_).advertise("image_rect", 1);
                }

                break;
            // publish monochrome rectified left image and disparity image from the camera
            case PROC_MODE_DISPARITY:
                left_camera_pub_ = image_transport::ImageTransport(left_nh_).advertiseCamera("image_rect", 1);
                disparity_pub_ = nh_.advertise<stereo_msgs::DisparityImage>("disparity", 1);
                break;
        }

        if (current_config_.calculate_points) { cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("points", 1); }
        if (current_config_.calculate_points2) { cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points2", 1); }
    }

    void cleanup()
    {
        if (stcam_)
        {
            stcam_->stop();
            delete stcam_;
        }

        stcam_->fini();
    }

    ~VidereStereoNode()
    {
        cleanup();
    }

    void calculate_points()
    {
        cloud_.points.resize(stcam_->stIm->numPts);
        cloud_.channels.resize(3);

        cloud_.channels[0].name = "rgb";
        cloud_.channels[0].values.resize(stcam_->stIm->numPts);
        cloud_.channels[1].name = "u";
        cloud_.channels[1].values.resize(stcam_->stIm->numPts);
        cloud_.channels[2].name = "v";
        cloud_.channels[2].values.resize(stcam_->stIm->numPts);

        for (int i = 0; i < stcam_->stIm->numPts; i++)
        {
            cloud_.points[i].x = stcam_->stIm->imPts[3*i + 0];
            cloud_.points[i].y = stcam_->stIm->imPts[3*i + 1];
            cloud_.points[i].z = stcam_->stIm->imPts[3*i + 2];
        }

        for (int i = 0; i < stcam_->stIm->numPts; i++)
        {
            int rgb = (stcam_->stIm->imPtsColor[3*i] << 16) | (stcam_->stIm->imPtsColor[3*i + 1] << 8) | stcam_->stIm->imPtsColor[3*i + 2];
            cloud_.channels[0].values[i] = *(float*)(&rgb);
            cloud_.channels[1].values[i] = stcam_->stIm->imCoords[2*i+0];
            cloud_.channels[2].values[i] = stcam_->stIm->imCoords[2*i+1];
        }
    }

    void calculate_points2()
    {
        // Fill in sparse point cloud message
        cloud2_.height = stcam_->stIm->imHeight;
        cloud2_.width  = stcam_->stIm->imWidth;
        cloud2_.fields.resize (4);
        cloud2_.fields[0].name = "x";
        cloud2_.fields[0].offset = 0;
        cloud2_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        cloud2_.fields[1].name = "y";
        cloud2_.fields[1].offset = 4;
        cloud2_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        cloud2_.fields[2].name = "z";
        cloud2_.fields[2].offset = 8;
        cloud2_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        cloud2_.fields[3].name = "rgb";
        cloud2_.fields[3].offset = 12;
        cloud2_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

        //cloud2_.is_bigendian = false; ???
        cloud2_.point_step = 16;
        cloud2_.row_step = cloud2_.point_step * cloud2_.width;
        cloud2_.data.resize (cloud2_.row_step * cloud2_.height);
        cloud2_.is_dense = true;

        for (int i = 0; i < stcam_->stIm->numPts; i++)
        {
            // x,y,z,rgba
            memcpy (&cloud2_.data[i * cloud2_.point_step + 0], &stcam_->stIm->imPts[3*i + 0], sizeof (float));
            memcpy (&cloud2_.data[i * cloud2_.point_step + 4], &stcam_->stIm->imPts[3*i + 1], sizeof (float));
            memcpy (&cloud2_.data[i * cloud2_.point_step + 8], &stcam_->stIm->imPts[3*i + 2], sizeof (float));
        }

        // Fill in color
        for (int i = 0; i < stcam_->stIm->numPts; i++)
        {
            int32_t rgb_packed = (stcam_->stIm->imPtsColor[3*i] << 16) | (stcam_->stIm->imPtsColor[3*i + 1] << 8) | stcam_->stIm->imPtsColor[3*i + 2];
            memcpy (&cloud2_.data[i * cloud2_.point_step + 12], &rgb_packed, sizeof (int32_t));
        }
    }

    void fillHeaders()
    {
        ros::Time timestamp = stcam_->stIm->left_raw.header.stamp;

        //-------------- populate header timestamps -------------------------//

        image_.header.stamp = timestamp;

        // disparity and clouds
        cloud_.header.stamp = timestamp;
        cloud2_.header.stamp = timestamp;

        // diagnostics
        timestamp_diag_.tick(timestamp);

        //-------------- populate header frame_ids ---------------------------//

        // camera infos
        stcam_->stIm->left_info.header.frame_id = current_config_.frame_id;
        stcam_->stIm->right_info.header.frame_id = current_config_.frame_id;

        // raw images
        stcam_->stIm->left_raw.header.frame_id = current_config_.frame_id;
        stcam_->stIm->right_raw.header.frame_id = current_config_.frame_id;

        image_.header.frame_id = current_config_.frame_id;

        // disparity and clouds
        stcam_->stIm->img_disp.image.header.frame_id = current_config_.frame_id;
        cloud_.header.frame_id = current_config_.frame_id;
        cloud2_.header.frame_id = current_config_.frame_id;
    }

    bool serviceCam()
    {
        if (!stcam_->getImage(100 + (int)(1.0/desired_freq_ * 1000)))
        {
            ROS_WARN("Timed out waiting for camera.");
            return false;
        }

        fillHeaders();

        left_camera_pub_.publish(stcam_->stIm->left_raw, stcam_->stIm->left_info);

        switch (videre_mode_)
        {
            // Publish left and right raw images, color information is available
            case PROC_MODE_TEST:
            case PROC_MODE_OFF:
            case PROC_MODE_NONE:
                right_camera_pub_.publish(stcam_->stIm->right_raw, stcam_->stIm->right_info);

                if (current_config_.convert_to_color)
                {
                    processor_.process(boost::make_shared<const sensor_msgs::Image>(stcam_->stIm->left_raw),
                                       stcam_->stIm->stereo_model.left(), left_set_, image_proc::Processor::ALL);
                    processor_.process(boost::make_shared<const sensor_msgs::Image>(stcam_->stIm->right_raw),
                                       stcam_->stIm->stereo_model.right(), right_set_, image_proc::Processor::ALL);

                    // publish all left processed images
                    publishImage(left_color_image_pub_, left_set_.color, left_set_.color_encoding);
                    publishImage(left_color_rect_image_pub_, left_set_.rect_color, left_set_.color_encoding);
                    publishImage(left_mono_image_pub_, left_set_.mono, sensor_msgs::image_encodings::MONO8);
                    publishImage(left_mono_rect_image_pub_, left_set_.rect, sensor_msgs::image_encodings::MONO8);

                    // publish all right processed images
                    publishImage(right_color_image_pub_, right_set_.color, right_set_.color_encoding);
                    publishImage(right_color_rect_image_pub_, right_set_.rect_color, right_set_.color_encoding);
                    publishImage(right_mono_image_pub_, right_set_.mono, sensor_msgs::image_encodings::MONO8);
                    publishImage(right_mono_rect_image_pub_, right_set_.rect, sensor_msgs::image_encodings::MONO8);
                }

                break;
            // publish left and right rectified mono images, color information is NOT available
            case PROC_MODE_RECTIFIED:
                right_camera_pub_.publish(stcam_->stIm->right_raw, stcam_->stIm->right_info);
                break;
            // publish left raw image and disparity image, color information is available
            case PROC_MODE_DISPARITY_RAW:
                if (current_config_.convert_to_color)
                {
                    processor_.process(boost::make_shared<const sensor_msgs::Image>(stcam_->stIm->left_raw),
                                       stcam_->stIm->stereo_model.left(), left_set_, image_proc::Processor::ALL);

                    // publish all left processed images
                    publishImage(left_color_image_pub_, left_set_.color, left_set_.color_encoding);
                    publishImage(left_color_rect_image_pub_, left_set_.rect_color, left_set_.color_encoding);
                    publishImage(left_mono_image_pub_, left_set_.mono, sensor_msgs::image_encodings::MONO8);
                    publishImage(left_mono_rect_image_pub_, left_set_.rect, sensor_msgs::image_encodings::MONO8);
                }

                if (stcam_->stIm->hasDisparity) { disparity_pub_.publish(stcam_->stIm->img_disp); }

//                 if (current_config_.calculate_points || current_config_.calculate_points2)
//                 {
//                     stcam_->stIm->doCalcPts(left_set_.rect_color, left_set_.color_encoding);
//                 }

                if (current_config_.calculate_points)
                {
//                     if (current_config_.calculate_points) { calculate_points(); }
                    stereo_processor_.processPoints(stcam_->stIm->img_disp, left_set_.rect_color,
                                                    left_set_.color_encoding, stcam_->stIm->stereo_model,
                                                    cloud_);
                    cloud_pub_.publish(cloud_);
                }

                if (current_config_.calculate_points2)
                {
                    // if (current_config_.calculate_points2) { calculate_points2(); }
                    stereo_processor_.processPoints2(stcam_->stIm->img_disp, left_set_.rect_color,
                                                     left_set_.color_encoding, stcam_->stIm->stereo_model,
                                                     cloud2_);
                    cloud2_pub_.publish(cloud2_);
                }

                break;
            // publish left mono image and disparity image, color information is NOT available
            case PROC_MODE_DISPARITY:
                if (stcam_->stIm->hasDisparity) { disparity_pub_.publish(stcam_->stIm->img_disp); }

                if (current_config_.calculate_points || current_config_.calculate_points2)
                {
                    processor_.process(boost::make_shared<const sensor_msgs::Image>(stcam_->stIm->left_raw),
                                       stcam_->stIm->stereo_model.left(), left_set_, image_proc::Processor::RECT);
                    // stcam_->stIm->doCalcPts(left_set_.rect, sensor_msgs::image_encodings::MONO8);
                }

                if (current_config_.calculate_points)
                {
                    // if (current_config_.calculate_points) { calculate_points(); }
                    stereo_processor_.processPoints(stcam_->stIm->img_disp, left_set_.rect_color,
                                                    left_set_.color_encoding, stcam_->stIm->stereo_model,
                                                    cloud_);
                    cloud_pub_.publish(cloud_);
                }

                if (current_config_.calculate_points2)
                {
                    // if (current_config_.calculate_points2) { calculate_points2(); }
                    stereo_processor_.processPoints2(stcam_->stIm->img_disp, left_set_.rect_color,
                                                     left_set_.color_encoding, stcam_->stIm->stereo_model,
                                                     cloud2_);
                    cloud2_pub_.publish(cloud2_);
                }

                break;
        }

        ++count_;

        return true;
    }

    void publishImage(const image_transport::Publisher& pub, const cv::Mat& image, const std::string& encoding)
    {
        sensor_msgs::fillImage(image_, encoding, image.rows, image.cols, image.step, const_cast<uint8_t*>(image.data));
        pub.publish(image_);
    }

    void freqStatus(diagnostic_updater::DiagnosticStatusWrapper& status)
    {
        status.name = "Frequency Status";

        double freq = count_ / diagnostic_.getPeriod();

        if (freq < (0.9 * desired_freq_))
        {
            status.level = 2;
            status.message = "Desired frequency not met";
        }
        else
        {
            status.level = 0;
            status.message = "Desired frequency met";
        }

        status.values.resize(3);
        status.values[0].key = "Images in interval";
        status.values[0].value = boost::lexical_cast<std::string>(count_);
        status.values[1].key = "Desired frequency";
        status.values[1].value = boost::lexical_cast<std::string>(desired_freq_);
        status.values[2].key = "Actual frequency";
        status.values[2].value = boost::lexical_cast<std::string>(freq);

        count_ = 0;
    }

    bool spin()
    {
        ROS_INFO("Streaming...");

        // Start up the camera
        while (nh_.ok())
        {
            serviceCam();
            diagnostic_.update();
            ros::spinOnce();
        }

        return true;
    }
};

VidereStereoNode* g_sdc = NULL;

void sigsegv_handler(int sig)
{
    signal(SIGSEGV, SIG_DFL);
    printf("System segfaulted, stopping camera nicely\n");
    if (g_sdc) { g_sdc->cleanup(); }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "videre_stereo_node", ros::init_options::AnonymousName);

    g_sdc = new VidereStereoNode();
    g_sdc->spin();
    delete g_sdc;

    return 0;
}
