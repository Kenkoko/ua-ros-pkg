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

#include <cstdio>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <visualization_msgs/Marker.h>

#include <object_tracking/object.h>
#include <object_tracking/object_tracker.h>


#include <iomanip>
template<class T>
void print_matz(const cv::Mat &C)
{
    using namespace std;
    cout << setprecision( 3 ) << right << fixed;
    for ( int row = 0; row < C.rows; ++ row )
    {
        for ( int col = 0; col < C.cols; ++ col )
        {
            cout << setw( 5 ) << (double)C.at<T>(row, col) << " ";
        }
        cout << endl;
    }

}


ObjectTracker::ObjectTracker()
{
    initialized = false;
    fg_prob_threshold = 0.6;
    con_area_threshold = 40;
    cv::startWindowThread();

    fd = new cv::SiftFeatureDetector(cv::SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
                                     cv::SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());

    cv::SurfDescriptorExtractor extractor;
    cv::BruteForceMatcher<cv::L2<float> > matcher;
    de = new cv::VectorDescriptorMatch<cv::SurfDescriptorExtractor, cv::BruteForceMatcher<cv::L2<float> > >(extractor, matcher);
}

std::map<int, Contour>
ObjectTracker::find_objects(const cv::Mat& fg_prob_img,
                            const cv::Mat& camera_img,
                            ros::Time stamp)
{
    if (!initialized) { train_model(fg_prob_img, camera_img, stamp); initialized = true; }
    return find_known_objects(fg_prob_img, camera_img, stamp);
}

void ObjectTracker::train_model(const cv::Mat& fg_prob_img, const cv::Mat& original, ros::Time stamp)
{
    cv::Mat bin_image;
    cv::threshold(fg_prob_img, bin_image, fg_prob_threshold, 255, cv::THRESH_BINARY);
    bin_image.convertTo(bin_image, CV_8UC1);

    std::vector<std::vector<cv::Point> > contours;
    cv::Mat contour_bin_image = bin_image.clone();
    cv::findContours(contour_bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Make an HSV image
    cv::Mat hsv_img;
    cv::cvtColor(original, hsv_img, CV_BGR2HSV);

    // These vectors store information about the new objects
    std::vector<cv::Rect> fg_rects;
    std::vector<cv::RotatedRect> obj_rects;
    std::vector<cv::MatND> histograms;
    std::vector<cv::Mat> back_projects;
    std::vector<cv::Mat> masks;
    std::vector<double> areas;
    //std::vector<std::vector<cv::KeyPoint> > keypoints;
    //std::vector<cv::Mat> tr_imgs;

    // first thing is background model
    //back_projects.push_back(bg_neg_log_lik_img.clone());

    //cv::Mat bg_mask;
    //cv::bitwise_not(bin_image, bg_mask);
    //masks.push_back(bg_mask);

    int h_bins = 30, s_bins = 32;

    BOOST_FOREACH(Contour con, contours)
    {
        cv::Mat con_mat(con);
        double area = cv::contourArea(con_mat);

        if (area > con_area_threshold)
        {
            ROS_INFO("Found a possible object of size %f", area);
            areas.push_back(area);

            cv::Rect bounder = cv::boundingRect(con_mat);
            fg_rects.push_back(bounder);
            obj_rects.push_back(cv::minAreaRect(con_mat));

            cv::Mat mask = bin_image(bounder);
            cv::Mat hsv_roi = hsv_img(bounder);

            cv::MatND hist;

            int hist_size[] = {h_bins, s_bins};
            float hranges[] = {0, 180};
            float sranges[] = {0, 256};
            const float* ranges[] = {hranges, sranges};
            int channels[] = {0, 1};

            cv::calcHist(&hsv_roi, 1, channels, mask, hist, 2, hist_size, ranges);
            //cv::normalize(hist, hist, 1, 0, cv::NORM_L1);
            histograms.push_back(hist);

            cv::Mat obj_mask = cv::Mat::zeros(fg_prob_img.size(), CV_8UC1);
            cv::fillConvexPoly(obj_mask, con.data(), con.size(), cv::Scalar(255));
            masks.push_back(obj_mask);

            // Back project the histogram
            cv::Mat back_project;
            hsv_img.convertTo(hsv_img, CV_32F);
            cv::calcBackProject(&hsv_img, 1, channels, hist, back_project, ranges);
            //cv::log(bp_prob, bp_prob); // TODO to log or not to log?
            back_projects.push_back(back_project);

            // extract features
//             cv::Mat img1_orig = original(bounder);
//             cv::Mat img1;
//             cv::cvtColor(img1_orig, img1, CV_BGR2GRAY);
//
//             std::cout << std::endl << "< Extracting keypoints from object contour..." << std::endl;
//             std::vector<cv::KeyPoint> keypoints1;
//             fd->detect( img1, keypoints1 );
//             std::cout << keypoints1.size() << " >" << std::endl;
//             keypoints.push_back(keypoints1);
//             tr_imgs.push_back(img1.clone());
        }
    }

    size_t num_objects = back_projects.size();

    if (num_objects > 0)
    {
        objects.clear();
        std::vector<double> mins;
        std::vector<double> maxs;
        mins.resize(num_objects);
        maxs.resize(num_objects);

        stochastic_gradient_following(back_projects, masks, mins, maxs);

//         Object bg;
//         bg.id = 0;
//         bg.min = mins[0];
//         bg.max = maxs[0];
//         objects.push_back(bg);

        for (size_t i = 0; i < num_objects; ++i)
        {
            Object obj;
            obj.id = objects.size();
            obj.area = areas[i];
            obj.min = mins[i];
            obj.max = maxs[i];

            cv::Point center;
            center.x = (fg_rects[i].x + (fg_rects[i].width / 2));
            center.y = (fg_rects[i].y + (fg_rects[i].height / 2));
            obj.tracks.push_back(center);

            obj.timestamps.push_back(stamp);
            obj.histogram = histograms[i];
            obj.tight_bounding_box = obj_rects[i];

            //obj.keypoints = keypoints[i];
            //obj.tr_img = tr_imgs[i];

            objects.push_back(obj);
        }
    }
}

std::map<int, Contour>
ObjectTracker::find_known_objects(const cv::Mat& fg_prob_img,
                                  const cv::Mat& camera_img,
                                  ros::Time stamp)
{
    std::map<int, Contour> id_to_contour;
    if (objects.empty()) { ROS_INFO("No objects, doing nothing."); return id_to_contour; }

    cv::Mat original = camera_img.clone();

    // Find contours for all the blobs found by background subtraction
    std::vector<Contour> contours;
    cv::Mat bin_image = (fg_prob_img > fg_prob_threshold);
    cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Make an HSV image
    cv::Mat hsv_img;
    cv::cvtColor(original, hsv_img, CV_BGR2HSV);

    float hranges[] = {0, 180};
    float sranges[] = {0, 256};
    int channels[] = {0, 1};
    const float* ranges[] = {hranges, sranges};

    std::map<int, std::vector<Contour> > object_id_to_contours;

    // TODO: Initialize rectangles here

    BOOST_FOREACH(Contour con, contours)
    {
        cv::Mat con_mat(con);
        double area = cv::contourArea(con_mat);

        if (area > con_area_threshold)
        {
            cv::Rect bounder = cv::boundingRect(con_mat);
            cv::Mat back_projects(objects.size() + 1, bounder.area(), CV_32F);
            back_projects.row(0) = cv::Scalar(1);

            // TODO: Add each rect to list for publishing

//             cv::Mat bg_roi = fg_loglike_img(bounder).clone();
//             double min = objects[0].min;
//             double max = objects[0].max;
//             bg_roi.convertTo(bg_roi, bg_roi.type(), 1.0 / (max - min), -min / (max - min));
//             cv::Mat bg_row = bg_roi.reshape(1, 1);
//             cv::Mat row = back_projects.row(1);
//             bg_row.copyTo(row);

            cv::Mat hsv_roi = hsv_img(bounder);

            for (size_t i = 0; i < objects.size(); ++i)
            {
                cv::Mat back_project;
                cv::calcBackProject(&hsv_roi, 1, channels, objects[i].histogram, back_project, ranges);
                back_project.convertTo(back_project, CV_32F);

                double min = objects[i].min;
                double max = objects[i].max;
                back_project.convertTo(back_project, back_project.type(), 1.0 / (max - min), -min / (max - min));

                cv::namedWindow("BP_" + boost::lexical_cast<std::string>(i));
                cv::imshow("BP_" + boost::lexical_cast<std::string>(i), back_project);

                cv::Mat back_project_row = back_project.reshape(1, 1);
                cv::Mat rowi = back_projects.row(i + 1);
                back_project_row.copyTo(rowi);
            }

            cv::Mat phi = mlr_weights * back_projects;
            cv::exp(phi, phi);
            cv::Mat col_sums;
            cv::reduce(phi, col_sums, 0, 0, CV_32F);
            cv::divide(1.0, col_sums, col_sums);

            //skip last output because we don't care about the background prob
            for (int i = 0; i < phi.rows-1; ++i)
            {
                cv::Mat rowi = phi.row(i);
                cv::multiply(rowi, col_sums, rowi);
                cv::Mat img = rowi.reshape(1, bounder.height);
                cv::Mat bin_image = (img > fg_prob_threshold);
                //ROS_INFO("img %d", i);
                //print_matz<float>(img);

                cv::boxFilter(bin_image, bin_image, bin_image.type(), cv::Size(3,3));
//                 if (i > 0) // don't show background projection
                {
                    cv::namedWindow("objmlr_" + boost::lexical_cast<std::string>(i));
                    cv::imshow("objmlr_" + boost::lexical_cast<std::string>(i), img);
                    cv::namedWindow("Bin_" + boost::lexical_cast<std::string>(i));
                    cv::imshow("Bin_" + boost::lexical_cast<std::string>(i), bin_image);
                }

                std::vector<Contour> contours;
                cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(bounder.x, bounder.y));

                if (contours.empty()) { continue; }

                int max_index = 0;
                double max_area = 0;

                for (size_t j = 0; j < contours.size(); ++j)
                {
                    double area = cv::contourArea(cv::Mat(contours[j]));

                    if (area > max_area)
                    {
                        max_area = area;
                        max_index = j;
                    }
                }

                int id = objects[i].id;

                if (object_id_to_contours.find(id) == object_id_to_contours.end())
                {
                    object_id_to_contours[id] = std::vector<Contour>();
                }

                object_id_to_contours[id].push_back(contours[max_index]);

                //draw selected contour for debugging
                //cv::drawContours(original, object_id_to_contours[id], -1, CV_RGB(255, 0, 0));
            }
        }
    }

    for (size_t i = 0; i < objects.size(); ++i)
    {
        int id = objects[i].id;

        // a patch matching object with certain id was found in the current frame
        if (object_id_to_contours.find(id) != object_id_to_contours.end())
        {
            std::vector<Contour>& contours = object_id_to_contours[id];

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

            for (size_t j = 0; j < contours.size(); ++j)
            {
                cv::Rect r = cv::boundingRect(cv::Mat(contours[j]));
                double area = cv::contourArea(cv::Mat(contours[j]));
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

                // match features
//                 cv::Mat img2_orig = original(r);
//                 cv::Mat img2;
//                 cv::cvtColor(img2_orig, img2, CV_BGR2GRAY);
//
//                 cv::namedWindow("win" + boost::lexical_cast<std::string>(j));
//                 cv::imshow("win" + boost::lexical_cast<std::string>(j), img2);
//
//                 std::cout << std::endl << "< Extracting keypoints from second image..." << std::endl;
//                 std::vector<cv::KeyPoint> keypoints2;
//                 fd->detect( img2, keypoints2 );
//                 std::cout << keypoints2.size() << " >" << std::endl;
//
//                 // draw keypoints
//                 for(std::vector<cv::KeyPoint>::const_iterator it = keypoints2.begin(); it < keypoints2.end(); ++it )
//                 {
//                     cv::Point p(it->pt.x + r.x, it->pt.y + r.y);
//                     cv::circle(original, p, 3, CV_RGB(0, 255, 0));
//                 }
//
//                 std::cout << "< Computing and matching descriptors..." << std::endl;
//                 std::vector<int> matches;
//
//                 if( objects[i].keypoints.size() > 0 && keypoints2.size() > 0 )
//                 {
//                     de->clear();
//                     de->add( img2, keypoints2 );
//                     de->match( objects[i].tr_img, objects[i].keypoints, matches );
//                 }
//
//                 std::cout << ">" << std::endl;
//
//                 for (uint kk = 0; kk < matches.size(); ++kk)
//                 {
//                     std::printf("%d ", matches[kk]);
//                 }
//
//                 std::printf("\n");
            }

            // restrict search to last known location and vicinity
            int search_grid_size = 100 * std::pow(2, objects[i].missed_frames); // TODO

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
                id_to_contour[id] = contours[min_index];

                objects[i].tracks.push_back(min_center);
                objects[i].timestamps.push_back(stamp);
                objects[i].missed_frames = 0;
            }
        }
    }

    for (size_t i = 0; i < objects.size(); ++i)
    {
        int id = objects[i].id;
        if (id_to_contour.find(id) == id_to_contour.end()) { continue; }

        cv::putText(original,
                    boost::lexical_cast<std::string>(id),
                    cv::Point(objects[i].tracks.back().x, objects[i].tracks.back().y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.33, CV_RGB(255, 0, 0));

        if (objects[i].tracks.size() > 1)
        {
            for (size_t j = 1; j < objects[i].tracks.size(); ++j)
            {
                cv::line(original, objects[i].tracks[j-1], objects[i].tracks[j], track_colors[id % 5]);
            }
        }

        objects[i].tight_bounding_box = cv::minAreaRect(cv::Mat(id_to_contour[id]));
    }

    cv::namedWindow("objects");
    cv::imshow("objects", original);

    return id_to_contour;
}

void ObjectTracker::stochastic_gradient_following(std::vector<cv::Mat>& bp_prob,
                                                  std::vector<cv::Mat>& obj_mask,
                                                  std::vector<double>& mins,
                                                  std::vector<double>& maxs)
{
    int epochs = 15;
    double step_size = 0.1;
    int size = obj_mask[0].rows * obj_mask[0].cols;

    int num_objects = bp_prob.size();        // num of new objects
    int num_features = num_objects + 1;      // 1 = bias term

    // multinomial logisitc regression weights
    // the +1 at the end is for the "none of the above" category (i.e., background)
    mlr_weights.create(num_objects + 1, num_features, CV_32F);
    cv::randn(mlr_weights, cv::Scalar(0.0), cv::Scalar(0.1));

    // the +1 at the end is for the "none of the above" category (i.e., background)
    cv::Mat target_vectors = cv::Mat::zeros(num_objects + 1, size, CV_32F);
    cv::Mat feature_vectors(num_features, size, CV_32F);
    feature_vectors.row(0) = cv::Scalar(1); // The bias term

    for (int i = 0; i < num_objects; ++i)
    {
        cv::Mat bp = bp_prob[i].clone();                    // TODO: pass in reshaped back_projects?
        double* min = &mins[i];
        double* max = &maxs[i];
        //*min = 0.0; *max = 1.0;
        cv::minMaxLoc(bp, min, max);
        cv::normalize(bp, bp, 0, 1, cv::NORM_MINMAX);
        cv::Mat bp_vector = bp.reshape(1, 1);               // 1 channel, 1 row matrix [px_1 ... px_size]
        cv::Mat rowi = feature_vectors.row(i + 1);
        bp_vector.row(0).copyTo(rowi);

        // make mask a long vector and stack them
        cv::Mat mask_vector = obj_mask[i].reshape(1, 1);
        cv::normalize(mask_vector, mask_vector, 0.0f, 1.0f, cv::NORM_MINMAX, CV_32F);

        rowi = target_vectors.row(i);
        mask_vector.row(0).copyTo(rowi);
    }
    // Create the "none of the above" target
    cv::Mat st;
    cv::reduce(target_vectors, st, 0, 0, CV_32F);
    target_vectors.row(num_objects) = cv::Scalar(1.0);
    target_vectors.row(num_objects) -= st;

    // print_matz<float>(target_vectors);

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

            cv::Mat error = target_vectors.col(n) - ps;

            /*
            ROS_INFO("features");
            print_matz<float>(feat_col.t());

            ROS_INFO("target");
            print_matz<float>(target_vectors.col(n).t());

            ROS_INFO("prediction before");
            print_matz<float>(ps.t());

            ROS_INFO("error");
            print_matz<float>(error.t());
            */

            for (int k = 0; k < num_objects+1; ++k)
            {
                //cv::Mat rowk = last_delta.row(k);
                cv::Mat gradient = feat_col * step_size * error.at<float>(k,0);
                //cv::Mat delta = rowk * momentum + gradient.t() * (1 - momentum);
                //mlr_weights.row(k) = mlr_weights.row(k) + delta - weight_decay * mlr_weights.row(k);
                mlr_weights.row(k) = mlr_weights.row(k) + gradient.t();
                //delta.copyTo(rowk);
            }

            vals = mlr_weights * feat_col;

            cv::exp(vals, vals);
            sum = cv::sum(vals)[0];
            ps = vals * (1.0f / sum);

            //ROS_INFO("prediction after");
            //print_matz<float>(ps.t());
            //if(j == 500)
            //    exit(0);
        }

/*        float prediction_error = 0.0f;
        std::vector<cv::Mat> os;
        os.resize(num_objects+1);

        for (int j = 0; j < os.size(); ++j)
        {
            os[j] = cv::Mat(1, size, CV_32F);
        }

        for (int j = 0; j < size; ++j)
        {
            cv::Mat feat_col = feature_vectors.col(j);
            cv::Mat vals = mlr_weights * feat_col;

            cv::exp(vals, vals);
            double sum = cv::sum(vals)[0];
            cv::Mat ps = vals * (1.0 / sum);

            for (int k = 0; k < ps.rows; ++k)
            {
                os[k].at<float>(0, j) = ps.at<float>(k, 0);
            }

            cv::Mat diff;
            cv::absdiff(ps, target_vectors.col(j), diff);
            prediction_error += cv::sum(diff)[0];
        }

        for (int j = 0; j < os.size(); ++j)
        {
            cv::Mat m = os[j].reshape(1, obj_mask[0].rows);
            cv::namedWindow("mlr_" + boost::lexical_cast<std::string>(j));
            cv::imshow("mlr_" + boost::lexical_cast<std::string>(j), m);

            cv::Mat mask = target_vectors.row(j).reshape(1, obj_mask[0].rows);
            cv::namedWindow("mlr_masks_" + boost::lexical_cast<std::string>(j));
            cv::imshow("mlr_masks_" + boost::lexical_cast<std::string>(j), mask);

            cv::waitKey(1000);
        }

        ROS_INFO("Epoch [%d], error = %f\n", i, prediction_error / size);*/
    }

/*    ROS_INFO("sgd mlr_weights r = %d, c = %d\n", mlr_weights.rows, mlr_weights.cols);
    print_matz<float>(mlr_weights);*/
}
