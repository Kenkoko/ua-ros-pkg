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
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>

#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

class ObjectDetector
{

public:
  ObjectDetector(ros::NodeHandle& nh);
  virtual ~ObjectDetector();

  void image_callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::ImageConstPtr& fg_prob_msg);

private:
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> fg_prob_sub_;
  message_filters::Synchronizer<SyncPolicy>* sync_;

  sensor_msgs::CvBridge image_bridge_;
  sensor_msgs::CvBridge fg_prob_bridge_;
  sensor_msgs::CvBridge hist_bridge_;

  ros::ServiceClient add_object_client_;

};

#endif /* OBJECTDETECTOR_H_ */
