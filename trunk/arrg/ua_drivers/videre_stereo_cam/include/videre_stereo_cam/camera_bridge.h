/*********************************************************************
* Software License Agreement (BSD License)
*
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

#ifndef CAM_BRIDGE_HH
#define CAM_BRIDGE_HH

#include <opencv2/core/core.hpp>

#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>

#include <videre_stereo_cam/stereoimage.h>

namespace cam_bridge
{
    std::string ColorCodingToImageEncoding(color_coding_t coding)
    {
        using namespace sensor_msgs::image_encodings;

        if (coding == COLOR_CODING_MONO8)       return MONO8;
        if (coding == COLOR_CODING_MONO16)      return MONO16;
        if (coding == COLOR_CODING_BAYER8_RGGB) return BAYER_RGGB8;
        if (coding == COLOR_CODING_BAYER8_BGGR) return BAYER_BGGR8;
        if (coding == COLOR_CODING_BAYER8_GBRG) return BAYER_GBRG8;
        if (coding == COLOR_CODING_BAYER8_GRBG) return BAYER_GRBG8;
        if (coding == COLOR_CODING_RGB8)        return RGB8;
        if (coding == COLOR_CODING_RGBA8)       return RGBA8;

        ROS_WARN("cam_bridge: Don't know image encoding string for color coding %i", coding);
        return "";
    }

    void fill_img_msg(cam::ImageData* img_raw, sensor_msgs::Image& img_msg)
    {
        // @todo: this could all be less hard-coded
        if (img_raw->imRawType != COLOR_CODING_NONE)
        {
            std::string encoding = ColorCodingToImageEncoding(img_raw->imRawType);
            sensor_msgs::fillImage(img_msg, encoding, img_raw->imHeight, img_raw->imWidth, img_raw->imWidth, img_raw->imRaw);
        }
        else if (img_raw->imType != COLOR_CODING_NONE)
        {
            sensor_msgs::fillImage(img_msg, sensor_msgs::image_encodings::MONO8, img_raw->imHeight, img_raw->imWidth, img_raw->imWidth, img_raw->im);
        }
        else if (img_raw->imColorType != COLOR_CODING_NONE && img_raw->imColorType == COLOR_CODING_RGBA8)
        {
            sensor_msgs::fillImage(img_msg, sensor_msgs::image_encodings::RGBA8, img_raw->imHeight, img_raw->imWidth, 4 * img_raw->imWidth, img_raw->imColor);
        }
        else if (img_raw->imColorType != COLOR_CODING_NONE && img_raw->imColorType == COLOR_CODING_RGB8)
        {
            sensor_msgs::fillImage(img_msg, sensor_msgs::image_encodings::RGB8, img_raw->imHeight, img_raw->imWidth, 3 * img_raw->imWidth, img_raw->imColor);
        }
        else if (img_raw->imRectType != COLOR_CODING_NONE)
        {
            sensor_msgs::fillImage(img_msg, sensor_msgs::image_encodings::MONO8, img_raw->imHeight, img_raw->imWidth, img_raw->imWidth, img_raw->imRect);
        }
        else if (img_raw->imRectColorType != COLOR_CODING_NONE && img_raw->imRectColorType == COLOR_CODING_RGBA8)
        {
            sensor_msgs::fillImage(img_msg, sensor_msgs::image_encodings::RGBA8, img_raw->imHeight, img_raw->imWidth, 4 * img_raw->imWidth, img_raw->imRectColor);
        }
        else if (img_raw->imRectColorType != COLOR_CODING_NONE && img_raw->imRectColorType == COLOR_CODING_RGB8)
        {
            sensor_msgs::fillImage(img_msg, sensor_msgs::image_encodings::RGB8, img_raw->imHeight, img_raw->imWidth, 3 * img_raw->imWidth, img_raw->imRectColor);
        }
    }

    void fill_info_msg(cam::ImageData* img_raw, sensor_msgs::CameraInfo& info_msg)
    {
        info_msg.height = img_raw->imHeight;
        info_msg.width  = img_raw->imWidth;

        memcpy((char*)(&info_msg.D[0]), (char*)(img_raw->D),  5*sizeof(double));
        memcpy((char*)(&info_msg.K[0]), (char*)(img_raw->K),  9*sizeof(double));
        memcpy((char*)(&info_msg.R[0]), (char*)(img_raw->R),  9*sizeof(double));
        memcpy((char*)(&info_msg.P[0]), (char*)(img_raw->P), 12*sizeof(double));
    }

    void StereoDataToRawStereo(cam::StereoData* stIm,
                               sensor_msgs::Image& left_image,
                               sensor_msgs::Image& right_image,
                               sensor_msgs::CameraInfo& left_info,
                               sensor_msgs::CameraInfo& right_info,
                               stereo_msgs::DisparityImage& disparity_image)
    {
        ros::Time timestamp = ros::Time::now();

        // Populate image message
        left_image.header.stamp = timestamp;
        right_image.header.stamp = timestamp;

        fill_img_msg(stIm->imLeft, left_image);
        fill_img_msg(stIm->imRight, right_image);

        // Populate camera info message
        left_info.header.stamp = timestamp;
        right_info.header.stamp = timestamp;

        fill_info_msg(stIm->imLeft, left_info);
        fill_info_msg(stIm->imRight, right_info);

        if (stIm->hasDisparity)
        {
            disparity_image.header.stamp = timestamp;

            disparity_image.f = right_info.P[0];
            disparity_image.T = -right_info.P[3] / right_info.P[0];

            disparity_image.valid_window.x_offset = stIm->imDleft;
            disparity_image.valid_window.y_offset = stIm->imDtop;
            disparity_image.valid_window.width = stIm->imDwidth;
            disparity_image.valid_window.height = stIm->imDheight;

            disparity_image.min_disparity = 0.0;
            disparity_image.max_disparity = stIm->numDisp - 1.0;

            disparity_image.delta_d = 1.0 / stIm->dpp;

            // Fill in DisparityImage image data, converting to 32-bit float
            sensor_msgs::Image& dimage = disparity_image.image;
            dimage.height = stIm->imHeight;
            dimage.width = stIm->imWidth;
            dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            dimage.step = dimage.width * sizeof(float);
            dimage.data.resize(dimage.step * dimage.height);

            cv::Mat_<int16_t> disparity16(stIm->imHeight, stIm->imWidth, (int16_t *) &stIm->imDisp[0], stIm->imWidth * sizeof(int16_t));
            cv::Mat_<float> dmat(dimage.height, dimage.width, (float*)&dimage.data[0], dimage.step);

            // We convert from fixed-point to float disparity and also adjust for any x-offset between
            // the principal points: d = d_fp*inv_dpp - (cx_l - cx_r)
            disparity16.convertTo(dmat, dmat.type(), disparity_image.delta_d, -(left_info.K[2] - right_info.K[2]));
            ROS_ASSERT(dmat.data == &dimage.data[0]);
        }
    }
}

#endif
