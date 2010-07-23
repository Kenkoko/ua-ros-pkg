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

#include <visualization_msgs/Marker.h>

#include <object_tracking/object.h>
#include <object_tracking/known_object_finder.h>

using namespace std;

void print_mat_bzz2(CvMat *mat)
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


KnownObjectFinder::KnownObjectFinder()
{
    fg_prob_threshold = 0.6;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, .33, .33);
    cv::startWindowThread();
}

std::map<int, std::vector<cv::Point> > KnownObjectFinder::find_objects(const cv::Mat& neg_log_lik_img, const cv::Mat& lbp_foreground_img,
                                                                       const cv::Mat& camera_img, std::vector<Object>& objects, cv::Mat& mlr_weights,
                                                                       std::vector<cv::RotatedRect>& obj_rects)
{
    std::map<int, vector<cv::Point> > id_to_contour;

    // If no objects, just pass along the image unchanged - otherwise the other node has no input
    if (objects.empty())
    {
        ROS_INFO_STREAM("No objects, doing nothing.");
        return id_to_contour;
    }
    else
    {
        cv::Mat fg_loglike_img = neg_log_lik_img.clone();
        cv::Mat original = camera_img.clone();
        cv::Mat fg_lbp_img;
        lbp_foreground_img.convertTo(fg_lbp_img, CV_32F, 1.0 / 255);

        // Compute the (foreground) probability image under the logistic model
        double w = -1/5.0, b = 4.0;
        cv::Mat fg_prob_img = fg_loglike_img.clone();
        fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), w, b);
        cv::exp(fg_prob_img, fg_prob_img);
        fg_prob_img.convertTo(fg_prob_img, fg_prob_img.type(), 1, 1);
        cv::divide(1.0, fg_prob_img, fg_prob_img);

        cv::namedWindow("fg_prob_img");
        cv::imshow("fg_prob_img", fg_prob_img);

        ///////////////////////////////////////////////////////////////////
        /////  LBP integrates here (choose product or sum) ////////////////
        ///////////////////////////////////////////////////////////////////
        cv::namedWindow("fg_lbp_img");
        cv::imshow("fg_lbp_img", fg_lbp_img);

        cv::Mat combination_product;
        cv::multiply(fg_prob_img, fg_lbp_img, combination_product);

        cv::namedWindow("product");
        cv::imshow("product", combination_product);

        cv::Mat bin_image_prod = (combination_product > fg_prob_threshold);
        cv::namedWindow("binary_prod");
        cv::imshow("binary_prod", bin_image_prod);

        cv::Mat combination_sum;
        combination_sum = fg_prob_img * 0.5 + fg_lbp_img * 0.5;

        cv::namedWindow("sum");
        cv::imshow("sum", combination_sum);

        cv::Mat bin_image_sum = (combination_sum > fg_prob_threshold);
        cv::namedWindow("binary_sum");
        cv::imshow("binary_sum", bin_image_sum);
        ///////////////////////////////////////////////////////////////////

        // Find contours for all the blobs found by background subtraction
        std::vector<std::vector<cv::Point> > contours;
        cv::Mat bin_image = (fg_prob_img > fg_prob_threshold);
        // cv::imshow("binary", bin_image);

        cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        // Make an HSV image
        cv::Mat hsv_img = original.clone();
        cv::cvtColor(original, hsv_img, CV_BGR2HSV);

        float hranges[] = {0, 180};
        float sranges[] = {0, 256};
        int channels[] = {0, 1};
        const float* ranges[] = {hranges, sranges};

        std::map<int, vector<vector<cv::Point> > > obj_id_to_patches;

        // TODO: Initialize rectangles here

        BOOST_FOREACH(std::vector<cv::Point> con, contours)
        {
            double area = cv::contourArea(cv::Mat(con));

            if (area > 40)
            {
                cv::Rect bounder = cv::boundingRect(cv::Mat(con));
                cv::Mat back_projects(objects.size() + 1, bounder.area(), CV_32F);
                back_projects.row(0) = cv::Scalar(1);

                // TODO: Add each rect to list for publishing

                cv::Mat bg_roi = fg_loglike_img(bounder).clone();
                double min = objects[0].min;
                double max = objects[0].max;
                bg_roi.convertTo(bg_roi, bg_roi.type(), 1.0 / (max - min), -min / (max - min));
                cv::Mat bg_row = bg_roi.reshape(1, 1);
                cv::Mat row = back_projects.row(1);
                bg_row.copyTo(row);

                cv::Mat hsv_roi = hsv_img(bounder);

                for (size_t i = 1; i < objects.size(); ++i)
                {
                    cv::Mat back_project;
                    cv::calcBackProject(&hsv_roi, 1, channels, objects[i].histogram, back_project, ranges);
                    back_project.convertTo(back_project, CV_32F);

                    double min = objects[i].min;
                    double max = objects[i].max;
                    back_project.convertTo(back_project, back_project.type(), 1.0 / (max - min), -min / (max - min));

                    cv::Mat back_project_row = back_project.reshape(1, 1);
                    cv::Mat rowi = back_projects.row(i+1);
                    back_project_row.copyTo(rowi);
                }

                cv::Mat phi = mlr_weights * back_projects;
                cv::exp(phi, phi);
                cv::Mat col_sums;
                cv::reduce(phi, col_sums, 0, 0, CV_32F);
                cv::divide(1.0, col_sums, col_sums);

                for (int i = 0; i < phi.rows; ++i)
                {
                    cv::Mat rowi = phi.row(i);
                    cv::multiply(rowi, col_sums, rowi);
                    cv::Mat img = rowi.reshape(1, bounder.height);

                    cv::Mat bin_image = (img > 0.7);

                    cv::namedWindow("obj" + boost::lexical_cast<std::string>(i));
                    cv::imshow("obj" + boost::lexical_cast<std::string>(i), bin_image);

                    std::vector<std::vector<cv::Point> > contours;
                    cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(bounder.x, bounder.y));

                    if (contours.empty()) { continue; }

                    int max_index = 0;
                    double max_area = 0;

                    for (size_t j = 0; j < contours.size(); ++j)
                    {
                        std::vector<cv::Point>& con = contours[j];
                        double area = cv::contourArea(cv::Mat(con));

                        if (area > max_area)
                        {
                            max_area = area;
                            max_index = j;
                        }
                    }

                    int id = objects[i].id;

                    if (obj_id_to_patches.find(id) == obj_id_to_patches.end())
                    {
                        std::vector<std::vector<cv::Point> > v;
                        obj_id_to_patches[id] = v;
                    }

                    obj_id_to_patches[id].push_back(contours[max_index]);
                }
            }
        }

        for (size_t i = 1; i < objects.size(); ++i)
        {
            int id = objects[i].id;

            // a patch matching object with certain id was found in the current frame
            if (obj_id_to_patches.find(id) != obj_id_to_patches.end())
            {
                std::vector<std::vector<cv::Point> >& v = obj_id_to_patches[id];

                int min_index = 0;
                double min_dist = std::numeric_limits<double>::max();
                cv::Point min_center;

                cv::Point last_obj_loc = objects[i].tracks.back();
                size_t tracks_num = objects[i].tracks.size();

                if (tracks_num > 1)
                {
                    cv::Point p = objects[i].tracks[tracks_num - 1] - objects[i].tracks[tracks_num - 2];
                    last_obj_loc += p;
                }

                for (size_t j = 0; j < v.size(); ++j)
                {
                    cv::Rect r = cv::boundingRect(cv::Mat(v[j]));
                    double area = cv::contourArea(cv::Mat(v[j]));
                    cv::Point center(r.x + r.width / 2, r.y + r.height / 2);
                    cv::Point dist_diff = center - last_obj_loc;
                    double area_diff = area - objects[i].area;
                    double distance = sqrt(dist_diff.x * dist_diff.x + dist_diff.y * dist_diff.y + area_diff * area_diff);

                    if (distance < min_dist)
                    {
                        min_dist = distance;
                        min_index = j;
                        min_center = center;
                    }
                }

                // restrict search to last known location and vicinity
                int search_grid_size = 100; // * std::pow(2, objects[i].missed_frames); // TODO

                cv::Rect ar;
                ar.x = last_obj_loc.x - search_grid_size / 2;
                ar.y = last_obj_loc.y - search_grid_size / 2;
                ar.width = search_grid_size;
                ar.height = search_grid_size;

                if (!ar.contains(min_center))
                {
                    ++objects[i].missed_frames;
                }
                else
                {
                    id_to_contour[id] = v[min_index];

                    objects[i].tracks.push_back(min_center);
                    objects[i].missed_frames = 0;
                }
            }
        }

        IplImage orig_ipl = original;

        for (size_t i = 0; i < objects.size(); ++i)
        {
            int id = objects[i].id;
            if (id_to_contour.find(id) == id_to_contour.end()) { continue; }

            cvPutText(&orig_ipl, boost::lexical_cast<std::string>(id).c_str(), cvPoint(objects[i].tracks.back().x, objects[i].tracks.back().y), &font, CV_RGB(255, 0, 0));

            if (objects[i].tracks.size() > 1)
            {
                for (size_t j = 1; j < objects[i].tracks.size(); ++j)
                {
                    cv::line(original, objects[i].tracks[j-1], objects[i].tracks[j], CV_RGB(255, 0, 0));
                }
            }

            obj_rects.push_back(cv::minAreaRect(cv::Mat(id_to_contour[id])));
        }

        cv::namedWindow("objects");
        cv::imshow("objects", original);

        return id_to_contour;
    }
}
