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

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <object_tracking/object.h>
#include <object_tracking/known_object_finder.h>

using namespace std;

KnownObjectFinder::KnownObjectFinder()
{
    fg_prob_threshold = 0.6;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, .33, .33);
    cv::startWindowThread();
}

cv::Mat KnownObjectFinder::find_objects(const cv::Mat& neg_log_lik_img, const cv::Mat& camera_img, std::vector<Object> objects)
{
    // If no objects, just pass along the image unchanged - otherwise the other node has no input
    if (objects.empty())
    {
        ROS_INFO_STREAM("No objects, doing nothing.");
        return neg_log_lik_img;
    }
    else
    {
        ROS_INFO_STREAM("BEGIN");
        cv::Mat fg_loglike_img = neg_log_lik_img.clone();
        cv::Mat original = camera_img.clone();

        // Compute the (foreground) probability image under the logistic model
        double w = -1/5.0, b = 4.0;
        cv::Mat fg_prob_img = fg_loglike_img.clone();
        fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), w, b);
        cv::exp(fg_prob_img, fg_prob_img);
        fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), 1, 1);
        cv::divide(1.0, fg_prob_img, fg_prob_img);

        cv::namedWindow("fg_prob_img");
        cv::imshow("fg_prob_img", fg_prob_img);

        // Find contours for all the blobs found by background subtraction
        std::vector<std::vector<cv::Point> > contours;
        cv::Mat bin_image = (fg_prob_img > fg_prob_threshold);
        // cv::imshow("binary", bin_image);

        cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        // Make an HSV image
        cv::Mat hsv_img = original.clone();
        cv::cvtColor(original, hsv_img, CV_BGR2HSV);

        // cv::namedWindow("hsv_img");
        // cv::imshow("hsv_img", hsv_img);

        BOOST_FOREACH(std::vector<cv::Point> con, contours)
        {
            double area = cv::contourArea(cv::Mat(con));

            if (area > 40)
            {
                cv::Rect bounder = cv::boundingRect(cv::Mat(con));

            }
        }

        IplImage orig_ipl = original;

        BOOST_FOREACH(Object obj, objects)
        {
            cv::SparseMat& hist = obj.histogram;



            obj.subtract_self(fg_prob_img, original, bin_image, hsv_img, contours, fg_loglike_img);

            if (obj.wasFound)
            {
                cvPutText(&orig_ipl, boost::lexical_cast<std::string>(obj.id).c_str(), cvPoint(obj.tracks.back().x, obj.tracks.back().y), &font, CV_RGB(255, 0, 0));

                if (obj.tracks.size() > 1)
                {
                    for (size_t i = 1; i < obj.tracks.size(); ++i)
                    {
                        cv::line(original, obj.tracks[i-1], obj.tracks[i], CV_RGB(255, 0, 0));
                    }
                }
            }
        }

        cv::namedWindow("objects");
        cv::imshow("objects", original);

        ROS_INFO_STREAM("END");
        return fg_loglike_img;
    }
}
