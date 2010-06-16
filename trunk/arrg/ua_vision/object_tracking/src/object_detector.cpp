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

#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#include <boost/bind.hpp>

#include <background_filters/common.h>

using namespace std;

ObjectDetector::ObjectDetector(ros::NodeHandle& nh)
{
  // This is the original color image from the overhead camera
  image_sub_.subscribe(nh, "camera/image_color", 1);
  // This is foreground probability image with areas occupied by known objects blacked out
  fg_prob_sub_.subscribe(nh, "occluded_fg_objects", 1);

  // ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(10)
  sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), image_sub_, fg_prob_sub_);
  sync_->registerCallback(boost::bind(&ObjectDetector::image_callback, this, _1, _2));

  // This is the client for the service that tells the tracker a new object has appeared
  add_object_client_ = nh.serviceClient<object_tracking::AddObject> ("add_object");

  cv::startWindowThread();
}

ObjectDetector::~ObjectDetector()
{
  delete sync_;
}

void ObjectDetector::image_callback(const sensor_msgs::ImageConstPtr& image_msg,
                                    const sensor_msgs::ImageConstPtr& fg_prob_msg)
{
  // Convert the ROS Images to OpenCV
  cv::Mat original_big(image_bridge_.imgMsgToCv(image_msg));
  cv::Mat fg_prob_img(fg_prob_bridge_.imgMsgToCv(fg_prob_msg));

  cv::Mat original;
  cv::resize(original_big, original, fg_prob_img.size());

//  cout << image_msg->encoding << " " << fg_prob_img.type() << " " << CV_32FC1 << " " << fg_prob_img.size().height << " " << fg_prob_img.size().width << endl;

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
  cv::threshold(fg_prob_img, bin_image, 0.6, 1.0, cv::THRESH_BINARY);
  bin_image.convertTo(bin_image, CV_8UC1);

  cv::namedWindow("binary");
  cv::imshow("binary", bin_image);

  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(bin_image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  // Make an HSV image
  cv::Mat hsv_img = original.clone();
  cv::cvtColor(original, hsv_img, CV_BGR2HSV);

//  cv::namedWindow("hsv_img");
//  cv::imshow("hsv_img", hsv_img);

  srand(time(NULL));

  // These vectors store information about the new objects
  std::vector<cv::Rect> fg_rects;
  std::vector<cv::MatND> histograms;
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
    if (area > 100)
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

//      // Back project the histogram
//      cv::Mat back_project;
//      cv::calcBackProject(&hsv_img, 1, channels, hist, back_project, ranges);
//
//      // Convert the 0-255 "probabilities" to float probs (might be able to hack this away later)
//      cv::Mat bp_prob;
//      back_project.convertTo(bp_prob, CV_32FC1, 1.0/255.0);

//      cv::namedWindow("TEST");
//      cv::imshow("TEST", bp_prob);

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

    cout << "CALLING SERVICE..." << endl;
    add_object_client_.call(srv);
    cout << "... SERVICE COMPLETE" << endl;

//    cv::waitKey(10000000);
  }

//  cv::namedWindow("contours");
//  cv::imshow("contours", original);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_detector", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  ObjectDetector detector(n);
  ros::spin();

  return 0;
}
