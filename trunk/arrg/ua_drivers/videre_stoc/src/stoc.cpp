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

#include "stoc.h"
#include <SVS/svsclass.h>

//! Macro for throwing an exception with a message
#define STOC_EXCEPT(except, msg) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "[STOC::%s]: " msg, __FUNCTION__); \
    throw except(buf); \
  }

//! Macro for throwing an exception with a message, passing args
#define STOC_EXCEPT_ARGS(except, msg, ...) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "[STOC::%s]: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf); \
  }

stoc::STOC::STOC () : capture_type_(-1), format_(-1), channel_(-1), swap_mode_(false), color_mode_(0),
//                      color_alg_(2), proc_mode_(PROC_MODE_RECTIFIED), rate_(15), frame_div_(-1), size_w_(640), size_h_(480),
                      color_alg_(2), proc_mode_(PROC_MODE_NONE), rate_(15), frame_div_(-1), size_w_(640), size_h_(480),
                      rectification_(1), z_max_(5), multiproc_en_(true), cut_di_(64), speckle_diff_(-1), 
                      speckle_size_(400), horopter_(-1), corrsize_(15), unique_(10), tex_thresh_(5), ndisp_(64), 
                      debug_(true)
{
  process_   = new svsStereoProcess ();  // Compute the disparity image, and 3D points
  multiproc_ = new svsMultiProcess ();   // Multiscale processing
  video_     = getVideoObject ();        // Get access to the video stream
  // Set the intrinsic camera parameters (found after calibration)
  intrinsic_  = cvCreateMat (3, 3, CV_64FC1);
  distortion_ = cvCreateMat (1, 4, CV_64FC1);
  cvmSet (intrinsic_, 0, 0, 429.609702); cvmSet (intrinsic_, 0, 1, 0.0);        cvmSet (intrinsic_, 0, 2, 309.444590);
  cvmSet (intrinsic_, 1, 0, 0.0);        cvmSet (intrinsic_, 1, 1, 429.070702); cvmSet (intrinsic_, 1, 2, 222.472089);
  cvmSet (intrinsic_, 2, 0, 0.0);        cvmSet (intrinsic_, 2, 1, 0.0);        cvmSet (intrinsic_, 2, 2, 1.0);
  cvmSet (distortion_, 0, 0, -0.006479); cvmSet (distortion_, 0, 1, 0.039474);  cvmSet (distortion_, 0, 2, 0.0); cvmSet (distortion_, 0, 3, 0.0);
}

stoc::STOC::~STOC ()
{
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device
void
  stoc::STOC::readParametersFromFile (const char* parameter_file)
{
  if (parameter_file != NULL)
  {
    video_->ReadParams ((char*)parameter_file);
    fprintf (stderr, ">> Using camera parameters from %s\n", parameter_file);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device
void
  stoc::STOC::sendInternalParameters ()
{
   if (capture_type_ != -1)
  {
    if (video_->SetCapture (capture_type_))
    { if (debug_) fprintf (stderr, "[STOC] >> Set capture type to %d\n", capture_type_); }
    else
      STOC_EXCEPT_ARGS(stoc::Exception, "Error while setting capture type to %d!", capture_type_);
  }

  if (format_ != -1)
  {
    if (video_->SetFormat (format_))
    { if (debug_) fprintf (stderr, "[STOC] >> Set format type to %d\n", format_); }
    else
      STOC_EXCEPT_ARGS(stoc::Exception, "Error setting format type to %d!", format_);
  }

  if (channel_ != -1)
  {
    if (video_->SetChannel (channel_))
    { if (debug_) fprintf (stderr, "[STOC] >> Set channel type to %d\n", channel_); }
    else
      STOC_EXCEPT_ARGS(stoc::Exception, "Error setting channel type to %d!", channel_);
  }

  if (swap_mode_)
  {
    if (video_->SetSwap (swap_mode_))
    { if (debug_) fprintf (stderr, "[STOC] >> Swap mode enabled\n"); }
    else
      STOC_EXCEPT(stoc::Exception, "Error enabling swap mode!");
  }

  if (color_mode_ != -1)
  {
    bool r = false;
    if (color_mode_ == 0) r = video_->SetColor (true, true);
    if (color_mode_ == 1) r = video_->SetColor (true, false);
    if (color_mode_ == 2) r = video_->SetColor (false, true);
    if (r & debug_)
      fprintf (stderr, "[STOC] >> Color mode set to %d\n", color_mode_);
    else
      STOC_EXCEPT_ARGS(stoc::Exception, "Error setting color mode to %d!", color_mode_);
  }

  if (color_alg_ != -1)
  {
    if (video_->SetColorAlg (color_alg_))
    { if (debug_) fprintf (stderr, "[STOC] >> Color algorithm set to %d\n", color_alg_); }
    else
      STOC_EXCEPT_ARGS(stoc::Exception, "Error setting color algorithm to %d!", color_alg_);
  }

  if (video_->SetSize (size_w_, size_h_))
  { if (debug_) fprintf (stderr, "[STOC] >> Image size set to %dx%d\n", size_w_, size_h_); }
  else
      STOC_EXCEPT_ARGS(stoc::Exception, "Error setting image size to %dx%d!", size_w_, size_h_);

  if (frame_div_ != -1)
  {
    if (video_->SetFrameDiv (frame_div_))
    { if (debug_) fprintf (stderr, "[STOC] >> Image sampling set to %d\n", frame_div_); }
    else
      STOC_EXCEPT_ARGS(stoc::Exception, "Error setting image sampling to %d!", frame_div_);
  }

  if (rate_ != -1)
  {
    if (video_->SetRate (rate_))
    { if (debug_) fprintf (stderr, "[STOC] >> Image rate set to %d\n", rate_); }
    else
      STOC_EXCEPT_ARGS(stoc::Exception, "Error setting image rate to %d!", rate_);
  }

  if (proc_mode_ != -1)
  {
    if (video_->SetProcMode (proc_mode_type(proc_mode_)))
    { if (debug_) fprintf (stderr, "[STOC] >> Processing mode set to %d\n", proc_mode_); }
    else
      STOC_EXCEPT_ARGS(stoc::Exception, "Error setting STOC processing mode to %d!", proc_mode_);
  }

  if (rectification_ != -1)
  {
    if (video_->SetRect (rectification_))
    { if (debug_) fprintf (stderr, "[STOC] >> Image rectification set to %d\n", rectification_); }
    else
      STOC_EXCEPT_ARGS(stoc::Exception, "Error setting image rectification to %d!", rectification_);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device
void
  stoc::STOC::sendStereoParameters ()
{
  if (cut_di_ != 0 && debug_)
    fprintf (stderr, "[STOC] >> Disconsidering the last %d lines from the bottom of the disparity image...\n", cut_di_);
  if (ndisp_ != -1)
  {
    video_->SetNDisp (ndisp_);
    if (debug_) fprintf (stderr, "[STOC] >> Number of disparities set to %d\n", ndisp_);
  }
  if (tex_thresh_ != -1)
  {
    video_->SetThresh (tex_thresh_);
    if (debug_) fprintf (stderr, "[STOC] >> Texture filter threshold set to %d\n", tex_thresh_);
  }
  if (unique_ != -1)
  {
    video_->SetUnique (unique_);
    if (debug_) fprintf (stderr, "[STOC] >> Uniqueness filter threshold set to %d\n", unique_);
  }
  if (corrsize_ != -1)
  {
    video_->SetCorrsize (corrsize_);
    if (debug_) fprintf (stderr, "[STOC] >> Correlation window size set to %d\n", corrsize_);
  }
  if (horopter_ != -1)
  {
    video_->SetHoropter (horopter_);
    if (debug_) fprintf (stderr, "[STOC] >> Horopter (X-Offset) value set to %d\n", horopter_);
  }
  if (speckle_size_ != -1)
  {
    video_->SetSpeckleSize (speckle_size_);
    if (debug_) fprintf (stderr, "[STOC] >> Minimum disparity region size set to %d\n", speckle_size_);
  }
  if (speckle_diff_ != -1)
  {
    video_->SetSpeckleDiff (speckle_diff_);
    if (debug_) fprintf (stderr, "[STOC] >> Disparity region neighbor diff to %d\n", speckle_diff_);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set up the device
int
  stoc::STOC::open ()
{
  int res;
  int nr_cam = video_->Enumerate ();  // Get the number of cameras available

  // No cameras found on the bus ? Search for one in the taxi...
  if (nr_cam == 0)
  {
    STOC_EXCEPT(stoc::Exception, "No cameras found!");
    return (-1);
  }

  if (debug_)
    for (int i = 0; i < nr_cam; i++)
      fprintf (stderr, "[STOC] > Found camera %d: %s\n", i, video_->DeviceIDs ()[i]);

  // ---[ Open the camera ]---
  res = video_->Open (nr_cam - 1);  // Open camera
  device_id_ = video_->DeviceIDs ()[nr_cam - 1];

  if (!res)
  {
    STOC_EXCEPT(stoc::Exception, "Failed to open camera port!");
    return (-1);
  }

  res = video_->ReadParams ();
  if (!res)
  {
    STOC_EXCEPT(stoc::Exception, "Could not query camera for intrinsic parameters!");
    return (-1);
  }
  
  sendInternalParameters ();
  video_->binning = 1;

  sendStereoParameters ();
  
  // Start video streaming
  res = video_->Start ();
  if (!res)
    STOC_EXCEPT(stoc::Exception, "Failed to start data acquisition!");

  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Shutdown the device
int
  stoc::STOC::close ()
{
  int res = video_->Stop ();   // Stop video streaming
  // ---[ Close the camera ]---
  res = video_->Close ();
  if (!res)
    return (-1);
  return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Undistort an image based on the intrinsic camera parameters
void
  stoc::STOC::undistort (uint8_t *img, uint8_t *un_img, int width, int height)
{
  CvMat *src = cvCreateMatHeader (height, width, CV_8UC3);
  cvSetData (src, img, width * 3);
  CvMat *dst = cvCreateMatHeader (height, width, CV_8UC3);
  cvSetData (dst, un_img, width * 3);
  
  cvUndistort2 (src, dst, intrinsic_, distortion_);
}


////////////////////////////////////////////////////////////////////////////////
// Read data from the device
void stoc::STOC::readData (sensor_msgs::PointCloud &cloud, sensor_msgs::Image &left, sensor_msgs::Image &right, stereo_msgs::DisparityImage &disparity)
{
  svsStereoImage *si = video_->GetImage (10); // 10 ms timeout
  if (si == NULL && debug_)
    STOC_EXCEPT(stoc::Exception, "No image, timed out...");
    
  // Compute the disparity image
  if (multiproc_en_)
    multiproc_->CalcStereo (si);
  else
    process_->CalcStereo (si);

  // Trim down those nasty points at the bottom of the disparity image (errors?)
  for (int i = si->ip.height - cut_di_; i < si->ip.height; i++)
    for (int j = 0; j < si->ip.width; j++)
      si->disparity[i * si->ip.width + j] = -2;

  // Compute the 3D point cloud information
  if (multiproc_en_)
    multiproc_->Calc3D (si, 0, 0, 0, 0, NULL, NULL, z_max_);
  else
    process_->Calc3D (si, 0, 0, 0, 0, NULL, NULL, z_max_);

  // Save the 3D point cloud data in the structure, if present
  int nr_points = 0;

  // Fill in the ROS PointCloud message
  cloud.points.resize (si->ip.height * si->ip.width);            // allocate more than needed, we will resize later
  cloud.channels.resize (1);
  cloud.channels[0].name = "i";
  cloud.channels[0].values.resize (cloud.points.size ());
  
  // RGB or monochrome ?
  if (si->haveColor)
  {
    cloud.channels.resize (3);
    cloud.channels[0].name = "r";
    cloud.channels[0].values.resize (cloud.points.size ());        // re-do channels[0], just in case
    cloud.channels[1].name = "g";
    cloud.channels[1].values.resize (cloud.points.size ());
    cloud.channels[2].name = "b";
    cloud.channels[2].values.resize (cloud.points.size ());
  }

  //fprintf (stderr, "[STOC] >> Right before have3D\n");
  
  if (si->have3D)
  {
    svs3Dpoint *pts = si->pts3D;
    for (int i = 0; i < si->ip.height; i++)
    {
      for (int j = 0; j < si->ip.width; j++, pts++)
      {
        // Check if the point is valid
        if (pts->A <= 0)
          continue;

        cloud.points[nr_points].x = pts->X;
        cloud.points[nr_points].y = pts->Y;
        cloud.points[nr_points].z = pts->Z;

        if (si->haveColor)
        {
          svsColorPixel *mpc = (svsColorPixel*)(si->color + (i*si->ip.width + j) * 4);
          cloud.channels[0].values[nr_points] = mpc->r;  // red
          cloud.channels[1].values[nr_points] = mpc->g;  // green
          cloud.channels[2].values[nr_points] = mpc->b;  // blue
        }
        else
          cloud.channels[0].values[nr_points] = (unsigned char)si->Left ()[i*si->ip.width + j];

        nr_points++;
      } // width
    } // height
  } // have3D

  cloud.points.resize (nr_points);
  cloud.channels[0].values.resize (nr_points);

  // RGB or monochrome ?
  if (si->haveColor)
  {
    cloud.channels[1].values.resize (nr_points);
    cloud.channels[2].values.resize (nr_points);
  }

  //fprintf (stderr, "[STOC] >> Right before Left channel\n");
  
  // Prepare the left channel
  left.width  = si->ip.width;
  left.height = si->ip.height;
  left.step = si->ip.width * 1; // 1 byte in mono8 image row
  left.set_data_size(si->ip.width * si->ip.height);

  //fprintf (stderr, "[STOC] >> Just set width, height, step in left channel\n");
  
  // Check if <left> has color or monochrome
  if (si->haveColor)
  {
    left.set_data_size(left.get_data_size() * 3);  // RGB
    left.step = si->ip.width * 3;
    left.encoding  = "rgb8";
    unsigned char *in_left = (unsigned char*)si->Color ();
    
    // Stores a copy of the left image
    uint8_t *img_tmp = (uint8_t*)malloc (si->ip.width * si->ip.height * 3);

    for (int i = 0, j = 0; i < si->ip.width * si->ip.height; i++, j+=3, in_left +=4)
    {
      img_tmp[j + 0] = in_left[0];
      img_tmp[j + 1] = in_left[1];
      img_tmp[j + 2] = in_left[2];
    }
    memcpy (&(left.data[0]), img_tmp, si->ip.width * si->ip.height * 3);
    free (img_tmp);
  }
  else
  {
    left.encoding  = "mono8";
    memcpy (&(left.data[0]), si->Left (), left.get_data_size ());
  }

  //fprintf (stderr, "[STOC] >> Right before Right channel\n");

  // Prepare the right channel
  right.width  = si->ip.width;
  right.height = si->ip.height;
  right.step = si->ip.width * 1; // 1 byte in mono8 image row
  right.set_data_size(si->ip.width * si->ip.height);

  // Check if <right> has color or monochrome
  if (si->haveColorRight)
  {
    right.set_data_size (right.get_data_size() * 3);  // RGB
    right.step = si->ip.width * 3;
    right.encoding  = "rgb8";
    unsigned char *in_right = (unsigned char*)si->ColorRight ();
    for (int i = 0, j = 0; i < si->ip.width * si->ip.height; i++, j+=3, in_right +=4)
    {
      right.data[j + 0] = in_right[0];
      right.data[j + 1] = in_right[1];
      right.data[j + 2] = in_right[2];
    }
  }
  else
  {
    right.encoding  = "mono8";
    memcpy (&(right.data[0]), si->Right (), right.get_data_size ());
  }

  // Prepare the disparity channel
  if (si->haveDisparity)
  {
    disparity.image.width  = si->dp.dwidth;
    disparity.image.height = si->dp.dheight;
    disparity.image.encoding  = "mono16";
    disparity.f = (4.6 * 1000 / 6);
//    disparity.cx = 0;
//    disparity.cy = 0;
//    disparity.Tx = 0.09;
    disparity.T = 0.09;
    disparity.image.step = si->dp.dwidth * 2;
    disparity.image.set_data_size (si->dp.dwidth * si->dp.dheight * 2);
    memcpy (&(disparity.image.data[0]), si->disparity, disparity.image.get_data_size());
  }

  ros::Time t = ros::Time::now();

  cloud.header.stamp = t;
  cloud.header.frame_id = "stereo_cloud";
  left.header.stamp = t;
  right.header.stamp = t;
  disparity.header.stamp = t;

  return;
}

