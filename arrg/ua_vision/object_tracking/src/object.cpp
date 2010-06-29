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

#include <geometry_msgs/Point.h>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <background_filters/common.h>
#include <object_tracking/object.h>

void Object::subtract_self(const cv::Mat& fg_prob_img, const cv::Mat& orig_img, const cv::Mat& bin_img,
                           const cv::Mat& hsv_img, std::vector<std::vector<cv::Point> > contours, cv::Mat& fg_loglike_img)
{
    int h_bins = 30;
    int s_bins = 32;
    // int hist_size[] = {h_bins, s_bins};
    float hranges[] = {0, 180};
    float sranges[] = {0, 256};
    const float* ranges[] = {hranges, sranges};
    int channels[] = {0, 1};


    // Back project the histogram
//      cv::Mat back_project;
//      cv::calcBackProject(&hsv_img, 1, channels, histogram, back_project, ranges);

    // Convert the 0-255 "probabilities" to float probs (might be able to hack this away later)
//      cv::Mat bp_prob;
//      back_project.convertTo(bp_prob, CV_32FC1, 1.0/255.0);

//      cv::namedWindow("TEST");
//      cv::imshow("TEST", bp_prob);

    // Multiply the back-projected probabilities with the foreground probabilities
//      cv::Mat fg_combined_prob;
//      cv::multiply(fg_prob_img, bp_prob, fg_combined_prob);

    wasFound = false;

    BOOST_FOREACH(std::vector<cv::Point> contour, contours)
    {
        cv::Rect bounder = cv::boundingRect(cv::Mat(contour));

        cv::Point new_center(0, 0);
        cv::Point last = tracks.back();

        if (tracks.size() > 1)
        {
            cv::Point last2 = tracks[tracks.size() - 2];
            new_center.x = last.x - last2.x;
            new_center.y = last.y - last2.y;
        }

        new_center += last;
        cv::Rect new_bounder = bounder + cv::Size(bounder.width / 2, bounder.height / 2);

        if (new_center.inside(new_bounder))
        {
            cv::Mat roi = hsv_img(bounder);

            // Back project the histogram
            cv::Mat back_project;
            cv::calcBackProject(&roi, 1, channels, histogram, back_project, ranges);

            // Convert the 0-255 "probabilities" to float probs (might be able to hack this away later)
            cv::Mat bp_prob;
            back_project.convertTo(bp_prob, CV_32FC1, 1.0/255.0, 1.0/255.0);

            cv::Mat real_roi = fg_loglike_img(bounder);

            cv::log(bp_prob, bp_prob);
            cv::add(bp_prob, real_roi, bp_prob);

            double min, max;
            cv::minMaxLoc(bp_prob, &min, &max);
            ROS_INFO("min = %f, max = %f", min, max);

            bp_prob.convertTo(bp_prob, bp_prob.type(), -beta, -alpha);
            cv::exp(bp_prob, bp_prob);
            bp_prob.convertTo(bp_prob, bp_prob.type(), 1, 1);
            cv::divide(1.0, bp_prob, bp_prob);

            cv::Mat bin_image;
            cv::threshold(bp_prob, bin_image, 0.6, 255, cv::THRESH_BINARY);
            bin_image.convertTo(bin_image, CV_8UC1);

            std::vector<std::vector<cv::Point> > contours;
            cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            ROS_INFO("Found %Zu contours", contours.size());

            for (uint i = 0; i < contours.size(); ++i)
            {
                std::vector<cv::Point> con = contours[i];

                // If we have a reasonably large contour, we need to inform the tracker that it
                // is missing an object
                double area = cv::contourArea(cv::Mat(con));
                ROS_INFO("Contour %u has area %f", i, area);

                if (area > 20)
                {
                    bin_image.convertTo(bin_image, CV_32FC1);
                    bin_image.convertTo(bin_image, CV_32FC1, -1, 1);
                    cv::blur(bin_image, bin_image, cv::Size(3, 3));
                    cv::multiply(bin_image, real_roi, real_roi);

                    cv::namedWindow("obj_" + boost::lexical_cast<std::string>(id));
                    cv::imshow("obj_" + boost::lexical_cast<std::string>(id), bp_prob);

                    ROS_INFO("Object %d found itself at location (%d, %d)", id, tracks.back().x, tracks.back().y);
                    wasFound = true;

                    cv::Rect b = cv::boundingRect(cv::Mat(con));
                    int cx = b.x + b.width / 2 + bounder.x;
                    int cy = b.y + b.height / 2 + bounder.y;
                    cv::Point center(cx, cy);

                    tracks.push_back(center);
                }
            }

            if (contours.size() > 0) { break; }
        }
    }
}
