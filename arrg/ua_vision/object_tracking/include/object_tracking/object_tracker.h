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

#ifndef KNOWNOBJECTFINDER_H_
#define KNOWNOBJECTFINDER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <object_tracking/object.h>

static cv::Scalar track_colors[5] = { CV_RGB(255,   0,   0),
                                      CV_RGB(255, 116,   0),
                                      CV_RGB(  0, 153, 153),
                                      CV_RGB(255, 240,  48),
                                      CV_RGB(160,  15,  81),
                                    };

typedef std::vector<cv::Point> Contour;

class ObjectTracker
{
private:
    bool initialized;
    double fg_prob_threshold;
    double con_area_threshold;

    cv::FeatureDetector* fd;
    cv::GenericDescriptorMatch* de;

    cv::Mat mlr_weights;

    void find_new_objects(const cv::Mat& bg_neg_log_lik_img, const cv::Mat& camera_img, ros::Time stamp);

    std::map<int, Contour>
    find_known_objects(const cv::Mat& neg_log_lik_img,
                       const cv::Mat& lbp_foreground_img,
                       const cv::Mat& camera_img,
                       ros::Time stamp);

    void stochastic_gradient_following(std::vector<cv::Mat>& bp_prob,
                                       std::vector<cv::Mat>& obj_mask,
                                       std::vector<double>& mins,
                                       std::vector<double>& maxs);

public:
    std::vector<Object> objects;

    ObjectTracker();

    std::map<int, Contour>
    find_objects(const cv::Mat& neg_log_lik_img,
                 const cv::Mat& lbp_foreground_img,
                 const cv::Mat& camera_img,
                 ros::Time stamp);
};

#endif
