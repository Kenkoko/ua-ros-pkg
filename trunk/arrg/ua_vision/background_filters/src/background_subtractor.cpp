/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Antons Rebguns.
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

#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <background_filters/background_subtractor.h>

#define TWO_PI 6.28318531

using namespace std;

void BackgroundSubtractor::initialize(const std::string colorspace,
                                      const cv::Mat avg_img,
                                      const std::vector<float> cov_mats_inv,
                                      const std::vector<float> dets)
{
    this->colorspace = colorspace;
    this->avg_img = avg_img;
    this->img_n_chan = avg_img.channels();
    this->cov_mats_inv = cov_mats_inv;
    this->dets = dets;

    partition.resize(dets.size());
    double coef = pow(TWO_PI, img_n_chan / 2.0);

    for (size_t i = 0; i < dets.size(); ++i)
    {
        partition[i] = 1.0 / (coef * sqrt(dets[i]));
    }
}

template <class T>
cv::Mat BackgroundSubtractor::difference(const cv::Mat& new_img)
{
    cv::Mat prob_img(new_img.size(), CV_32FC1);
    float *prob_data = prob_img.ptr<float>();

    int height = new_img.rows;
    int width = new_img.cols;

    cv::Mat bgr_new(1, img_n_chan, CV_32FC1);
    cv::Mat bgr_ave(1, img_n_chan, CV_32FC1);
    cv::Mat inv_cov;

    for (int row = 0; row < height; ++row)
    {
        T* ptr_bg = new_img.ptr<T>(row);
        T* ptr_ave = avg_img.ptr<T>(row);

        for (int col = 0; col < width; ++col)
        {
            int pixel = row * width + col;

            for (int ch = 0; ch < img_n_chan; ++ch)
            {
                bgr_new.at<float>(0, ch) = ptr_bg[img_n_chan*col + ch];
                bgr_ave.at<float>(0, ch) = ptr_ave[img_n_chan*col + ch];
            }

            inv_cov = cv::Mat(img_n_chan, img_n_chan, CV_32FC1, &cov_mats_inv[pixel*(img_n_chan*img_n_chan)], sizeof(float)*img_n_chan);

            double mah_dist = cv::Mahalanobis(bgr_new, bgr_ave, inv_cov);
            double unnorm_gaussian = exp(-0.5 * mah_dist);
            float p = partition[pixel] * unnorm_gaussian;

            prob_data[pixel] = p;
        }
    }

    // calculate negative log-likelihood, darker areas are background, lighter - objects
    cv::log(prob_img, prob_img);
    prob_img.convertTo(prob_img, prob_img.type(), -1.0);

    return prob_img;
}

cv::Mat BackgroundSubtractor::subtract_background(const cv::Mat& new_img)
{
    if (colorspace == "rgb" || colorspace == "hsv")
    {
        return difference<const uchar>(new_img);
    }
    else if (colorspace == "rgchroma")
    {
        return difference<const float>(new_img);
    }
    else
    {
        printf("[BackgroundSubtractor] Unrecognized colorspace [%s]\n", colorspace.c_str());
        return cv::Mat();
    }
}
