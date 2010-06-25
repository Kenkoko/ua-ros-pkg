/*
 * ObjectDetector.cpp
 *
 *  Created on: Jun 15, 2010
 *      Author: Daniel Hewlett
 */

#include <ros/ros.h>

#include <object_tracking/object_detector.h>
#include <object_tracking/AddObject.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/bind.hpp>

#include <background_filters/common.h>

using namespace std;

ObjectDetector::ObjectDetector(ros::NodeHandle& nh)
{
  // This is foreground probability image with areas occupied by known objects blacked out
  fg_prob_sub_ = image_transport::ImageTransport(nh).subscribe("occluded_fg_objects", 1, image_callback());

  // This is the client for the service that tells the tracker a new object has appeared
  add_object_client_ = nh.serviceClient<object_tracking::AddObject> ("add_object");

  cv::startWindowThread();
}

void ObjectDetector::image_callback(const sensor_msgs::ImageConstPtr& fg_prob_msg)
{

  // Convert the ROS Images to OpenCV
  cv::Mat original_big(image_bridge_.imgMsgToCv(image_msg));
  cv::Mat bg_neg_log_lik_img(fg_prob_bridge_.imgMsgToCv(fg_prob_msg));

  cv::Mat original;
  cv::resize(original_big, original, bg_neg_log_lik_img.size());

  cv::Mat fg_prob_img = bg_neg_log_lik_img.clone();

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
    object_tracking::AddObject srv;

    srv.request.area = areas[i];
    srv.request.center.x = (fg_rects[i].x + (fg_rects[i].width / 2.0));
    srv.request.center.y = (fg_rects[i].y + (fg_rects[i].height / 2.0));

//    srv.request.roi.height = fg_rects[i].height;
//    srv.request.roi.width = fg_rects[i].width;
//    srv.request.roi.x_offset = fg_rects[i].x;
//    srv.request.roi.y_offset = fg_rects[i].y;



//    displayHist(histograms[i], "MatND");
    cv::SparseMat hist_sparse_mat(histograms[i]);
//    displayHist(hist_sparse_mat, "SparseMat");
    cv::Mat hist_mat;
    hist_sparse_mat.copyTo(hist_mat);
    IplImage ipl = hist_mat;
    srv.request.histogram = (*sensor_msgs::CvBridge::cvToImgMsg(&ipl));
    srv.request.histogram.header = image_msg->header;
    srv.request.alpha = alphas[i];
    srv.request.beta = betas[i];

    cout << "CALLING SERVICE..." << endl;
    add_object_client_.call(srv);
    cout << "... SERVICE COMPLETE" << endl;

//    cv::waitKey(10000000);
  }

//  cv::namedWindow("contours");
//  cv::imshow("contours", original);

}

void ObjectDetector::sgd(vector<cv::Mat&> bp_prob, const cv::Mat& log_lik_ratio, const cv::Mat& obj_mask, double& alpha, double& beta, int id)
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
    int size = 320*240;   //TODO: change those
    vector<double> mins;
    vector<double> maxs;
    mins.resize(num_objects);
    maxs.resize(num_objects);

    // multinomial logisitc regression weights
    cv::Mat_<float> mlr_weights(num_objects, num_features);
    cv::Mat_<float> last_delta = cv::Mat_<float>::zeros(num_objects, num_features);
    float momentum = 0.9;
    float weight_decay = 0.001;

    cv::Mat_<float> feature_vectors(size, num_features);
    feature_vectors.row(0) = cv::Scalar(1);

    for (int i = 1; i < num_objects; ++i)
    {
        cv::Mat bp = bp_prob[i-1];
        cv::minMaxLoc(bp, &mins[i-1], &maxs[i-1]);
        cv::normalize(bp, bp, 0, 1, cv::NORM_MINMAX);
        bp.reshape(1, 1);   // 1 channel, 1 row matrix [px_1 ... px_size]
        bp.row(0).copyTo(feature_vectors.row(i));
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
            double sum = cv::sum(vals);
            cv::Mat ps = vals * (1.0 / sum);

            for (int k = 0; k < num_objects; ++k)
            {
                cv::Mat gradient = feat_col * (step_size * (label - ps[k]));
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


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detector", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  ObjectDetector detector(n);
  ros::spin();

  return 0;
}
