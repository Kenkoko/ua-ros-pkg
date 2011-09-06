/*
    Copyright (c) <year>, <copyright holder>
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY <copyright holder> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef OBJECT_H
#define OBJECT_H

#include <fstream>

#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

class Object
{
public:
    int id;
    double area;
    double min;
    double max;

    int missed_frames;

    std::vector<cv::Point> tracks;
    std::vector<ros::Time> timestamps;

    cv::MatND histogram;
    cv::RotatedRect tight_bounding_box;

    std::vector<cv::Mat> lbp_region_histograms;
    //cv::Mat tr_img;
    //std::vector<cv::KeyPoint> keypoints;

    Object() : missed_frames(0)
    {
    }

    void update_histogram(cv::Mat hsv_img, cv::Mat bin_image)
    {
        cv::MatND hist;

        cv::Rect bounder = tight_bounding_box.boundingRect();

        cv::Mat mask = bin_image(bounder);
        cv::Mat hsv_roi = hsv_img(bounder);

        int h_bins = 30, s_bins = 32;
        int hist_size[] = {h_bins, s_bins};
        float hranges[] = {0, 180};
        float sranges[] = {0, 256};
        const float* ranges[] = {hranges, sranges};
        int channels[] = {0, 1};

        cv::calcHist(&hsv_roi, 1, channels, mask, hist, 2, hist_size, ranges);
        cv::addWeighted(hist, 0.1, histogram, 0.9, 0.0, histogram);
    }

    void set_lbp_region_histograms(const cv::Mat& current_lbp)
    {
        lbp_region_histograms = compute_lbp_region_histograms(current_lbp);
        ROS_INFO("[%d] Number of LBP region histograms is %d", id, (int)lbp_region_histograms.size());
    }

    void compare_lbp_histograms(const cv::Mat& current_lbp)
    {
        std::vector<cv::Mat> current_lbp_region_histograms = compute_lbp_region_histograms(current_lbp);
        ROS_INFO("[%d] Number of current LBP region histograms is %d", id, (int)lbp_region_histograms.size());
    }

    void dump_to_file()
    {
        std::string t = boost::lexical_cast<std::string>(ros::Time::now().sec) + "." + boost::lexical_cast<std::string>(ros::Time::now().nsec);
        std::string fname = "tracks_" + boost::lexical_cast<std::string>(id) + "_" + t + ".dat";
        std::ofstream f(fname.c_str());

        for (size_t i = 0; i < tracks.size(); ++i)
        {
            f << id << "," << timestamps[i] << "," << tracks[i].x << "," << tracks[i].y << std::endl;
        }
    }

private:
    std::vector<cv::Mat> compute_lbp_region_histograms(const cv::Mat& current_lbp)
    {
        std::vector<cv::Mat> region_hists;

        int region_out = 3;
        int region_step = region_out; // could take step sizes different from region half-size

        for (int r = region_out; r <= current_lbp.rows-region_out; r+= region_step)
        {
            for (int c = region_out; c <= current_lbp.cols-region_out; c+= region_step)
            {
                // Calculate the current region histogram
                cv::Mat cur_hist = cv::Mat::zeros(1, 256, CV_8U);
                uint8_t* h = cur_hist.ptr<uint8_t>(0);

                for (int region_row = r-region_out; region_row < r+region_out+1; ++region_row)
                {
                    const uint8_t* rowptr = current_lbp.ptr<const uint8_t>(region_row);

                    for (int region_col = c-region_out; region_col < c+region_out+1; ++region_col)
                    {
                        uint8_t val = rowptr[region_col];
                        h[val]++;
                    }
                }

                region_hists.push_back(cur_hist);
            }
        }

        return region_hists;
    }
};

#endif // OBJECT_H
