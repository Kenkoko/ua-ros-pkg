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

#include <ros/ros.h>

#include <object_tracking/new_object_finder.h>
#include <object_tracking/AddObject.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/bind.hpp>

#include <background_filters/common.h>
#include <object_tracking/object.h>

using namespace std;

NewObjectFinder::NewObjectFinder()
{
    cv::startWindowThread();
}

void NewObjectFinder::find_objects(const cv::Mat& bg_neg_log_lik_img, const cv::Mat& camera_img, std::vector<Object>& objects)
{
    cv::Mat fg_prob_img = bg_neg_log_lik_img.clone();
    cv::Mat original = camera_img.clone();

    // Compute the (foreground) probability image under the logistic model
    double w = -1 / 5.0, b = 4.0;
    fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), w, b);
    cv::exp(fg_prob_img, fg_prob_img);
    fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), 1, 1);
    cv::divide(1.0, fg_prob_img, fg_prob_img);

    cv::namedWindow("fg_prob_img");
    cv::imshow("fg_prob_img", fg_prob_img);

    double max;
    cv::minMaxLoc(fg_prob_img, NULL, &max);
    ROS_INFO_STREAM(max);

    cv::Mat bin_image; // = (fg_prob_img > 0.6);
    cv::threshold(fg_prob_img, bin_image, 0.6, 255, cv::THRESH_BINARY);
    bin_image.convertTo(bin_image, CV_8UC1);

    cv::namedWindow("binary");
    cv::imshow("binary", bin_image);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    ROS_INFO("Found %Zu contours", contours.size());

    // Make an HSV image
    cv::Mat hsv_img = original.clone();
    cv::cvtColor(original, hsv_img, CV_BGR2HSV);

    //  cv::namedWindow("hsv_img");
    //  cv::imshow("hsv_img", hsv_img);

    srand(time(NULL));

    // These vectors store information about the new objects
    std::vector<cv::Rect> fg_rects;
    std::vector<cv::MatND> histograms;
    std::vector<double> alphas;
    std::vector<double> betas;
    std::vector<double> areas;

    int h_bins = 30, s_bins = 32;

    for (uint i = 0; i < contours.size(); ++i)
    {
        std::vector<cv::Point> con = contours[i];

        int r, g, b;
        r = rand() % 255;
        g = rand() % 255;
        b = rand() % 255;

        // If we have a reasonably large contour, we need to inform the tracker that it
        // is missing an object
        double area = cv::contourArea(cv::Mat(con));
        ROS_INFO("Contour %u has area %f", i, area);

        if (area > 40)
        {
            ROS_INFO_STREAM("FOUND AN OBJECT");

            std::vector<std::vector<cv::Point> > one_contour;
            one_contour.push_back(con);

            areas.push_back(area);

            // For debugging
            cv::drawContours(original, one_contour, -1, cv::Scalar(r, g, b));

            cv::Rect bounder = cv::boundingRect(cv::Mat(con));
            cv::rectangle(original, bounder, cv::Scalar(r, g, b));

            fg_rects.push_back(bounder);
            cv::Mat mask = bin_image(bounder);
            cv::Mat hsv_roi = hsv_img(bounder);

            cv::MatND hist;

            int hist_size[] = {h_bins, s_bins};
            // hue varies from 0 to 179, see cvtColor
            float hranges[] = {0, 180};
            // saturation varies from 0 (black-gray-white) to
            // 255 (pure spectrum color)
            float sranges[] = {0, 256};
            const float* ranges[] = {hranges, sranges};
            //                     int hist_size[] = {num_bins, num_bins};
            int channels[] = {0, 1};
            //                     float range[] = {0, 256};
            //                     const float* ranges[] = {range, range};
            cv::calcHist(&hsv_roi, 1, channels, mask, hist, 2, hist_size, ranges);
            histograms.push_back(hist);


            cv::Mat obj_mask = cv::Mat::zeros(bin_image.size(), bin_image.type());
            cv::fillConvexPoly(obj_mask, con.data(), con.size(), cv::Scalar(1));

            // Back project the histogram
            cv::Mat back_project;
            cv::calcBackProject(&hsv_img, 1, channels, hist, back_project, ranges);

            // Convert the 0-255 "probabilities" to float probs (might be able to hack this away later)
            cv::Mat bp_prob;
            back_project.convertTo(bp_prob, CV_32FC1, 1.0/255.0);
            cv::log(bp_prob, bp_prob);

            cv::Mat log_lik_ratio;
            cv::add(bp_prob, bg_neg_log_lik_img, log_lik_ratio);

            double alpha, beta;
            sgd(bp_prob, log_lik_ratio, obj_mask, alpha, beta, i);
            alphas.push_back(alpha);
            betas.push_back(beta);

            cv::normalize(log_lik_ratio, log_lik_ratio, 0, 1, cv::NORM_MINMAX);
            cv::namedWindow("hist " + boost::lexical_cast<string>(i));
            cv::imshow("hist " + boost::lexical_cast<string>(i), log_lik_ratio);
        }
    }

    for (uint i = 0; i < histograms.size(); ++i)
    {
        Object obj;

        obj.area = areas[i];

        cv::Point center;
        center.x = (fg_rects[i].x + (fg_rects[i].width / 2.0));
        center.y = (fg_rects[i].y + (fg_rects[i].height / 2.0));

        obj.tracks.push_back(center);
        obj.histogram = histograms[i];
        obj.alpha = alphas[i];
        obj.beta = betas[i];

        objects.push_back(obj);
    }

    //  cv::namedWindow("contours");
    //  cv::imshow("contours", original);
}

void NewObjectFinder::sgd(vector<cv::Mat>& bp_prob, const cv::Mat& log_lik_ratio, const cv::Mat& obj_mask, double& alpha, double& beta, int id)
{
    alpha = beta = 1;
    double step_size = 0.01;
    int epochs = 100;
    int size = obj_mask.rows * obj_mask.cols;
    int width = obj_mask.cols;

    cv::Mat log_lik_rat_norm = log_lik_ratio.clone();

    double min, max;
    cv::minMaxLoc(log_lik_rat_norm, &min, &max);
    cv::normalize(log_lik_rat_norm, log_lik_rat_norm, 0, 1, cv::NORM_MINMAX);

    int num_objects = 5 + 1;    // TODO: change those/ num objects + background
    int num_features = num_objects + 1; // 1 = bias term
    //int size = 320*240;   //TODO: change those
    vector<double> mins;
    vector<double> maxs;
    mins.resize(num_objects);
    maxs.resize(num_objects);

    // multinomial logisitc regression weights
    cv::Mat mlr_weights(num_objects, num_features, CV_32F);
    cv::Mat last_delta = cv::Mat::zeros(num_objects, num_features, CV_32F);
    float momentum = 0.9;
    float weight_decay = 0.001;

    cv::Mat feature_vectors(size, num_features, CV_32F);
    feature_vectors.row(0) = cv::Scalar(1);

    for (int i = 1; i < num_objects; ++i)
    {
        cv::Mat bp = bp_prob[i-1];
        cv::minMaxLoc(bp, &mins[i-1], &maxs[i-1]);
        cv::normalize(bp, bp, 0, 1, cv::NORM_MINMAX);
        bp.reshape(1, 1);   // 1 channel, 1 row matrix [px_1 ... px_size]
        cv::Mat rowi = feature_vectors.row(i);
        bp.row(0).copyTo(rowi);
    }

    for (int i = 0; i < epochs; ++i)
    {
        cv::Mat_<float> idx(1, size);

        for (int j = 0; j < size; j++)
        {
            idx(0, j) = j;
        }

        cv::randShuffle(idx);

        for (int j = 0; j < size; ++j)
        {
            int n = (int) idx(0, j);
            int row = n / width;
            int col = n % width;

            int label = obj_mask.at<uchar>(row, col);
            //float val = log_lik_rat_norm.at<float>(row, col);
            //float p = 1.0 / (1.0 + exp(-(alpha + beta * val)));   // binary variable logisitc regression

            cv::Mat feat_col = feature_vectors.col(j);
            cv::Mat vals = mlr_weights * feat_col;

            cv::exp(vals, vals);
            double sum = cv::sum(vals)[0];
            cv::Mat ps = vals * (1.0 / sum);

            for (int k = 0; k < num_objects; ++k)
            {
                cv::Mat gradient = feat_col * (step_size * (label - ps.at<float>(0, k)));
                cv::Mat delta = last_delta * momentum + gradient * (1 - momentum);
                mlr_weights.row(k) = mlr_weights.row(k) + delta - weight_decay * mlr_weights.row(k);
                last_delta = delta;
            }

            // alpha = alpha + step_size * (label - p);
            // beta = beta + step_size * (label - p) * val;
        }

        // printf("a = %f, b = %f\n", alpha, beta);
    }

    mlr_weights.col(0);

    for (int i = 1; i < mlr_weights.cols; ++i)
    {
        mlr_weights.col(i);
    }
/*    alpha = alpha - beta * (min / (max - min));
    beta = beta / (max - min);

    log_lik_ratio.convertTo(log_lik_rat_norm, log_lik_ratio.type(), -beta, -alpha);
    cv::exp(log_lik_rat_norm, log_lik_rat_norm);
    log_lik_rat_norm.convertTo(log_lik_rat_norm, log_lik_rat_norm.type(), 1, 1);
    cv::divide(1.0, log_lik_rat_norm, log_lik_rat_norm);
    cv::minMaxLoc(log_lik_rat_norm, &min, &max);
    printf("min = %f, max = %f\n", min, max);

    cv::namedWindow("object after " + boost::lexical_cast<string>(id));
    cv::imshow("object after " + boost::lexical_cast<string>(id), log_lik_rat_norm);*/
}
