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

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <object_tracking/object.h>
#include <object_tracking/new_object_finder.h>

void print_mat_bzz(CvMat *mat)
{
    for (int i = 0; i < mat->rows; i++)
    {
        std::printf("\n");

        switch (CV_MAT_DEPTH(mat->type))
        {
            case CV_32F:
            case CV_64F:
                for (int j = 0; j < mat->cols; j++)
                    std::printf("%8.6f ", (float)cvGetReal2D(mat, i, j));
                break;
            case CV_8U:
            case CV_16U:
                for (int j = 0; j < mat->cols; j++)
                    std::printf("%6d", (int)cvGetReal2D(mat, i, j));
                break;
            default:
                break;
        }
    }

    std::printf("\n");
}

NewObjectFinder::NewObjectFinder()
{
    cv::startWindowThread();
}

void NewObjectFinder::find_objects(const cv::Mat& bg_neg_log_lik_img, const cv::Mat& camera_img, std::vector<Object>& objects, cv::Mat& mlr_weights)
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
    cv::Mat contour_bin_image = bin_image.clone();
    cv::findContours(contour_bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
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
    std::vector<cv::Mat> back_projects;
    std::vector<cv::Mat> masks;
    std::vector<double> areas;

    // first thing is background model
    back_projects.push_back(bg_neg_log_lik_img.clone());

    cv::Mat bg_mask;
    cv::bitwise_not(bin_image, bg_mask);
    masks.push_back(bg_mask);

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

            cv::Mat obj_mask = cv::Mat::zeros(fg_prob_img.size(), CV_8UC1);
            cv::fillConvexPoly(obj_mask, con.data(), con.size(), cv::Scalar(255));
            masks.push_back(obj_mask);

            // Back project the histogram
            cv::Mat back_project;
            hsv_img.convertTo(hsv_img, CV_32F);
            cv::calcBackProject(&hsv_img, 1, channels, hist, back_project, ranges);

//             cv::log(bp_prob, bp_prob); // TODO to log or not to log?

            back_projects.push_back(back_project);
        }
    }

    int num_objects = back_projects.size();
    std::vector<double> alphas;
    std::vector<double> betas;
    alphas.resize(histograms.size());
    betas.resize(histograms.size());

    if (num_objects > 1)
    {
        objects.clear();
        std::vector<double> mins;
        std::vector<double> maxs;
        mins.resize(num_objects);
        maxs.resize(num_objects);

        sgd(back_projects, masks, mins, maxs, mlr_weights);

        Object bg;
        bg.id = objects.size();
        bg.min = mins[0];
        bg.max = maxs[0];
        objects.push_back(bg);

        for (uint i = 0; i < num_objects - 1; ++i)
        {
            Object obj;

            obj.area = areas[i];

            cv::Point center;
            center.x = (fg_rects[i].x + (fg_rects[i].width / 2.0));
            center.y = (fg_rects[i].y + (fg_rects[i].height / 2.0));

            obj.tracks.push_back(center);
            obj.histogram = histograms[i];
            obj.min = mins[i+1];
            obj.max = maxs[i+1];

            obj.id = objects.size();
            objects.push_back(obj);
        }

        //  cv::namedWindow("contours");
        //  cv::imshow("contours", original);
    }
}

void NewObjectFinder::sgd(std::vector<cv::Mat>& bp_prob, std::vector<cv::Mat>& obj_mask, std::vector<double>& mins, std::vector<double>& maxs, cv::Mat& mlr_return)
{
    int epochs = 5;
    double step_size = 0.1;
    float momentum = 0.;
    float weight_decay = 0.;
    int size = obj_mask[0].rows * obj_mask[0].cols;
    int width = obj_mask[0].cols;

    int num_objects = bp_prob.size();        // num of new objects + background
    int num_features = num_objects + 1;      // 1 = bias term

    // multinomial logisitc regression weights
    cv::Mat mlr_weights(num_objects, num_features, CV_32F);
    cv::randn(mlr_weights, cv::Scalar(0.0), cv::Scalar(0.1));
/*
    printf("mlr_after_init\n");
    print_mat_bzz(&((CvMat) mlr_weights));*/

    cv::Mat last_delta = cv::Mat::zeros(num_objects, num_features, CV_32F);
    cv::Mat mask_vectors(num_objects, size, CV_32F);
    cv::Mat feature_vectors(num_features, size, CV_32F);
    feature_vectors.row(0) = cv::Scalar(1);

    // first thing is background model
    for (int i = 0; i < num_objects; ++i)
    {
        cv::Mat bp = bp_prob[i].clone();                  // TODO: pass in reshaped back_projects?
        double* min = &mins[i];
        double* max = &maxs[i];
        cv::minMaxLoc(bp, min, max);
        cv::normalize(bp, bp, 0, 1, cv::NORM_MINMAX);
        cv::Mat bp_vector = bp.reshape(1, 1);                                   // 1 channel, 1 row matrix [px_1 ... px_size]
        cv::Mat rowi = feature_vectors.row(i + 1);
        bp_vector.row(0).copyTo(rowi);

        // make mask a long vector and stack them
        cv::Mat mask_vector = obj_mask[i].reshape(1, 1);
        cv::normalize(mask_vector, mask_vector, 0.0f, 1.0f, cv::NORM_MINMAX, CV_32F);

        rowi = mask_vectors.row(i);
        mask_vector.row(0).copyTo(rowi);
    }

    // initialize indexing array to be shuffled later
    cv::Mat_<int> idx(1, size);
    for (int j = 0; j < size; ++j)
    {
        idx(0, j) = j;
    }

    for (int i = 0; i < epochs; ++i)
    {
        // shuffle indexes
        cv::randShuffle(idx);

        for (int j = 0; j < size; ++j)
        {
            int n = idx(0, j);

            cv::Mat feat_col = feature_vectors.col(n);
            cv::Mat vals = mlr_weights * feat_col;

            cv::exp(vals, vals);
            float sum = cv::sum(vals)[0];
            cv::Mat ps = vals * (1.0f / sum);

            // debug
//             if (std::isinf<double>(sum))
//             {
//                 printf("sum = %f\nps:\n", sum);
//                 print_mat_bzz(&((CvMat) ps));
//                 printf("vals\n");
//                 print_mat_bzz(&((CvMat) vals));
//                 printf("mlr_weights\n");
//                 print_mat_bzz(&((CvMat) mlr_weights));
//                 printf("feat_col\n");
//                 print_mat_bzz(&((CvMat) feat_col));
//                 exit(1);
//             }
//
            for (int k = 0; k < num_objects; ++k)
            {
                cv::Mat rowk = last_delta.row(k);

/*                if (j < 100)
                {
                    printf("rowk before:\n");
                    print_mat_bzz(&((CvMat) rowk));
                }*/

                float error = mask_vectors.at<float>(k, n) - ps.at<float>(0, k);

//                 if (j < 100)
//                 {
//                     printf("%f - %f = error:%f\n", mask_vectors.at<float>(k, n), ps.at<float>(0, k), error);
//                 }

                cv::Mat gradient = feat_col * step_size * error;

/*                if (j < 100)
                {
                    printf("gradient:\n");
                    print_mat_bzz(&((CvMat) gradient));
                }*/

                cv::Mat delta = rowk * momentum + gradient.t() * (1 - momentum);

//                 if (j < 100)
//                 {
//                     printf("delta:\n");
//                     print_mat_bzz(&((CvMat) delta));
//                 }

                mlr_weights.row(k) = mlr_weights.row(k) + delta - weight_decay * mlr_weights.row(k);

//                 if (j < 100)
//                 {
//                     printf("mlr_weights.row(k):\n");
//                     print_mat_bzz(&((CvMat) mlr_weights.row(k)));
//                 }

                delta.copyTo(rowk);

/*                if (j < 100)
                {
                    printf("rowk after:\n");
                    print_mat_bzz(&((CvMat) rowk));
                }*/
            }
        }

        float prediction_error = 0.0f;

        for (int j = 0; j < size; ++j)
        {
            cv::Mat feat_col = feature_vectors.col(j);
            cv::Mat vals = mlr_weights * feat_col;

            cv::exp(vals, vals);
            double sum = cv::sum(vals)[0];
            cv::Mat ps = vals * (1.0 / sum);

            cv::Mat diff;
            cv::absdiff(ps, mask_vectors.col(j), diff);
            prediction_error += cv::sum(diff)[0];
        }

        printf("Epoch [%d], error = %f\n", i, prediction_error / size);
    }

    mlr_return = mlr_weights.clone();
    printf("sgd mlr_weights r = %d, c = %d\n", mlr_weights.rows, mlr_weights.cols);
}
