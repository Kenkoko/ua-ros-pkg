/*
 * ObjectDetector.h
 *
 *  Created on: Jun 15, 2010
 *      Author: Daniel Hewlett
 */

#ifndef OBJECTDETECTOR_H_
#define OBJECTDETECTOR_H_

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>

class ObjectDetector
{

public:
  ObjectDetector(ros::NodeHandle& nh);
  virtual ~ObjectDetector();

  void image_callback(const sensor_msgs::ImageConstPtr& fg_prob_msg);
  void sgd(std::vector< const const cv::Mat& > bp_prob, const cv::Mat& log_lik_ratio, const cv::Mat& obj_mask, double& alpha, double& beta, int id);

private:
  image_transport::Subscriber fg_prob_sub_;

  sensor_msgs::CvBridge image_bridge_;
  sensor_msgs::CvBridge fg_prob_bridge_;
  sensor_msgs::CvBridge hist_bridge_;

  ros::ServiceClient add_object_client_;

};

#endif /* OBJECTDETECTOR_H_ */
