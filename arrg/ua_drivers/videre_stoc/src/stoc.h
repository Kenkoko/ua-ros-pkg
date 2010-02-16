/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* author: Radu Bogdan Rusu <rusu@cs.tum.edu> */

#ifndef STOC_DRIVER_H
#define STOC_DRIVER_H

// ROS includes
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <stdexcept>
#include <string>
#include <SVS/svsclass.h>

#include <opencv/cv.h>

namespace stoc
{
  //! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
  // code borrowed from drivers/laser/hokuyo_driver/hokuyo.h
#define DEF_EXCEPTION(name, parent) \
  class name  : public parent { \
  public: \
    name (const char* msg) : parent (msg) {} \
  }
  
  //! A standard STOC exception
  DEF_EXCEPTION(Exception, std::runtime_error);

  class STOC
  {
    public:
      STOC ();
      ~STOC ();

      int open ();
      int close ();
      void readData (sensor_msgs::PointCloud &cloud, sensor_msgs::Image &left, sensor_msgs::Image &right, stereo_msgs::DisparityImage &disparity);

      void sendInternalParameters ();
      void sendStereoParameters ();
      void readParametersFromFile (const char* parameter_file = NULL);

      std::string device_id_;

      // Acquisition device calls (cannot be handled online through the ROS parameter server)
      int capture_type_, format_, channel_;
      bool swap_mode_;
      int color_mode_, color_alg_;
      
      // These must be set before streaming
      int proc_mode_, rate_, frame_div_, size_w_, size_h_;

      bool rectification_;
      float z_max_;

      bool multiproc_en_;
      int cut_di_;
      
      void setNDisp       (int n) { ndisp_        = n; video_->SetNDisp (ndisp_); if (debug_) ROS_INFO ("[STOC] >> Number of disparities set to %d", ndisp_); }
      void setTexThresh   (int n) { tex_thresh_   = n; video_->SetThresh (tex_thresh_); if (debug_) ROS_INFO ("[STOC] >> Texture filter threshold set to %d", tex_thresh_); }
      void setUnique      (int n) { unique_       = n; video_->SetUnique (unique_); if (debug_) ROS_INFO ("[STOC] >> Uniqueness filter threshold set to %d", unique_); }
      void setCorrSize    (int n) { corrsize_     = n; video_->SetCorrsize (corrsize_); if (debug_) ROS_INFO ("[STOC] >> Correlation window size set to %d", corrsize_); }
      void setHoropter    (int n) { horopter_     = n; video_->SetHoropter (horopter_); if (debug_) ROS_INFO ("[STOC] >> Horopter (X-Offset) value set to %d", horopter_); }
      void setSpeckleSize (int n) { speckle_size_ = n; video_->SetSpeckleSize (speckle_size_); if (debug_) ROS_INFO ("[STOC] >> Minimum disparity region size set to %d", speckle_size_); }
      void setSpeckleDiff (int n) { speckle_diff_ = n; video_->SetSpeckleDiff (speckle_diff_); if (debug_) ROS_INFO ("[STOC] >> Disparity region neighbor diff to %d", speckle_diff_); }
    private:

      int speckle_diff_, speckle_size_, horopter_, corrsize_, unique_, tex_thresh_, ndisp_;

      // SVS objects
      svsVideoImages   *video_;
      svsStereoProcess *process_;
      svsMultiProcess  *multiproc_;

      bool debug_;

      // Stores the intrinsic camera matrix and the distortion coefficients after calibration
      CvMat *intrinsic_, *distortion_;
      void undistort (uint8_t *img, uint8_t *un_img, int width, int height);
  };

};

#endif

