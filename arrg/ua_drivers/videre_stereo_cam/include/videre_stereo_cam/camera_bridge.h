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

#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>

#include "videre_stereo_cam/stereoimage.h"

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

  void CamDataToRawStereo(cam::ImageData* im, sensor_msgs::Image& im_msg, sensor_msgs::CameraInfo& info_msg)
  {
    // @todo: this could all be less hard-coded
    if (im->imRawType != COLOR_CODING_NONE)
    {
      std::string encoding = ColorCodingToImageEncoding(im->imRawType);
      sensor_msgs::fillImage(im_msg, encoding, im->imHeight, im->imWidth, im->imWidth, im->imRaw);
    }
    else if (im->imType != COLOR_CODING_NONE)
    {
      sensor_msgs::fillImage(im_msg, sensor_msgs::image_encodings::MONO8, im->imHeight, im->imWidth, im->imWidth, im->im);
    }
    else if (im->imColorType != COLOR_CODING_NONE && im->imColorType == COLOR_CODING_RGBA8)
    {
      sensor_msgs::fillImage(im_msg, sensor_msgs::image_encodings::RGBA8, im->imHeight, im->imWidth, 4 * im->imWidth, im->imColor);
    }
    else if (im->imColorType != COLOR_CODING_NONE && im->imColorType == COLOR_CODING_RGB8)
    {
      sensor_msgs::fillImage(im_msg, sensor_msgs::image_encodings::RGB8, im->imHeight, im->imWidth, 3 * im->imWidth, im->imColor);
    }
    else if (im->imRectType != COLOR_CODING_NONE)
    {
      sensor_msgs::fillImage(im_msg, sensor_msgs::image_encodings::MONO8, im->imHeight, im->imWidth, im->imWidth, im->imRect);
    }
    else if (im->imRectColorType != COLOR_CODING_NONE && im->imRectColorType == COLOR_CODING_RGBA8)
    {
      sensor_msgs::fillImage(im_msg, sensor_msgs::image_encodings::RGBA8, im->imHeight, im->imWidth, 4 * im->imWidth, im->imRectColor);
    }
    else if (im->imRectColorType != COLOR_CODING_NONE && im->imRectColorType == COLOR_CODING_RGB8)
    {
      sensor_msgs::fillImage(im_msg, sensor_msgs::image_encodings::RGB8, im->imHeight, im->imWidth, 3 * im->imWidth, im->imRectColor);
    }

    info_msg.height = im->imHeight;
    info_msg.width  = im->imWidth;

    memcpy((char*)(&info_msg.D[0]), (char*)(im->D),  5*sizeof(double));
    memcpy((char*)(&info_msg.K[0]), (char*)(im->K),  9*sizeof(double));
    memcpy((char*)(&info_msg.R[0]), (char*)(im->R),  9*sizeof(double));
    memcpy((char*)(&info_msg.P[0]), (char*)(im->P), 12*sizeof(double));
  }

  void StereoDataToRawStereo(cam::StereoData* stIm,
                             sensor_msgs::Image& left_image,
                             sensor_msgs::Image& right_image,
                             sensor_msgs::CameraInfo& left_info,
                             sensor_msgs::CameraInfo& right_info,
                             stereo_msgs::DisparityImage& disparity_image)
  {
    ros::Time timestamp = ros::Time().fromNSec(stIm->imLeft->im_time * 1000);
    
    left_image.header.stamp = timestamp;
    right_image.header.stamp = timestamp;
    left_info.header.stamp = timestamp;
    right_info.header.stamp = timestamp;
    
    CamDataToRawStereo(stIm->imLeft, left_image, left_info);
    CamDataToRawStereo(stIm->imRight, right_image, right_info);
    
    if (stIm->hasDisparity)
    {
      disparity_image.header.stamp = timestamp;
      
      sensor_msgs::fillImage(disparity_image.image,
                             sensor_msgs::image_encodings::TYPE_32FC1,
                             stIm->imWidth,
                             stIm->imHeight,
                             2 * stIm->imWidth,
                             stIm->imDisp);
                             
      disparity_image.f = right_info.P[0];
      disparity_image.T = -right_info.P[3] / right_info.P[0];
      
      disparity_image.valid_window.x_offset = stIm->imDleft;
      disparity_image.valid_window.y_offset = stIm->imDtop;
      disparity_image.valid_window.width = stIm->imDwidth;
      disparity_image.valid_window.height = stIm->imDheight;
      
      disparity_image.delta_d = 1.0 / stIm->dpp;
      
//      raw_stereo.disparity_info.dpp = stIm->dpp;
//      raw_stereo.disparity_info.num_disp = stIm->numDisp;
//      raw_stereo.disparity_info.corr_size = stIm->corrSize;
//      raw_stereo.disparity_info.filter_size = stIm->filterSize;
//      raw_stereo.disparity_info.hor_offset = stIm->horOffset;
//      raw_stereo.disparity_info.texture_thresh = stIm->textureThresh;
//      raw_stereo.disparity_info.unique_thresh = stIm->uniqueThresh;
//      raw_stereo.disparity_info.smooth_thresh = stIm->smoothThresh;
//      raw_stereo.disparity_info.speckle_diff = stIm->speckleDiff;
//      raw_stereo.disparity_info.speckle_region_size = stIm->speckleRegionSize;
//      raw_stereo.disparity_info.unique_check = stIm->unique_check;

//    } else {
//      clearImage(raw_stereo.disparity_image);
//      raw_stereo.has_disparity = false;
    }

//    memcpy((char*)(&raw_stereo.stereo_info.T[0]),  (char*)(stIm->T),   3*sizeof(double));
//    memcpy((char*)(&raw_stereo.stereo_info.Om[0]), (char*)(stIm->Om),  3*sizeof(double));
//    memcpy((char*)(&raw_stereo.stereo_info.RP[0]), (char*)(stIm->RP), 16*sizeof(double));
  }
}

#endif
