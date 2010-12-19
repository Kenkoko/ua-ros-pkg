/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef POINT_CLOUD_INTENSITY_FILTER_H
#define POINT_CLOUD_INTENSITY_FILTER_H
/**
\author Vijay Pradeep
@b ScanIntensityFilter takes input scans and fiters out that are not within the specified range. The filtered out readings are set at >max_range in order to invalidate them.

**/


#include "filters/filter_base.h"
#include "sensor_msgs/PointCloud.h"

namespace point_cloud_filters
{

class PointCloudIntensityFilter : public filters::FilterBase<sensor_msgs::PointCloud>
{
public:

  double lower_threshold_ ;
  double upper_threshold_ ;
  int disp_hist_ ;

  bool configure()
  {
    lower_threshold_ = 8000.0;
    upper_threshold_ = 100000.0;
    disp_hist_ = 1;
    getParam("lower_threshold", lower_threshold_);
    getParam("upper_threshold", upper_threshold_) ;
    getParam("disp_histogram",  disp_hist_) ;

    return true;
  }

  virtual ~PointCloudIntensityFilter()
  { 

  }

  bool update(const sensor_msgs::PointCloud& input_scan, sensor_msgs::PointCloud& filtered_scan)
  {
//    const double hist_max = 4*12000.0 ;
//    const int num_buckets = 24 ;
//    int histogram[num_buckets] ;
//    for (int i=0; i < num_buckets; i++)
//      histogram[i] = 0 ;
    filtered_scan = sensor_msgs::PointCloud();

    filtered_scan.header = input_scan.header;

    filtered_scan.points.reserve(input_scan.points.size());
    filtered_scan.channels.resize(input_scan.channels.size());

    int intensity_idx = -1;

    for (unsigned int i=0; i < filtered_scan.channels.size(); ++i)
    {
        filtered_scan.channels[i].name = input_scan.channels[i].name;
        filtered_scan.channels[i].values.reserve(input_scan.channels[i].values.size());
        if (input_scan.channels[i].name == "intensity") { intensity_idx = i; }
    }

    if (intensity_idx == -1)
    {
        ROS_WARN("intensity_filter: Intensity channel was not found in the point cloud");
        return false;
    }

    for (unsigned int i=0; i < input_scan.points.size(); ++i)
    {
      float intensity = input_scan.channels[intensity_idx].values[i];
      if (intensity <= lower_threshold_ || intensity >= upper_threshold_) { continue; }
      
      filtered_scan.points.push_back(input_scan.points[i]);
      
      for (unsigned int j = 0; j < input_scan.channels.size(); ++j)
      {
        filtered_scan.channels[j].values.push_back(input_scan.channels[j].values[i]);
      }

//      int cur_bucket = (int) ((filtered_scan.channels[intensity_idx].values[i]/hist_max)*num_buckets) ;
//      
//      if (cur_bucket >= num_buckets-1) { cur_bucket = num_buckets-1; }
//      histogram[cur_bucket]++ ;
    }

//    if (disp_hist_ > 0)                                                                 // Display Histogram
//    {
//      printf("********** CLOUD **********\n") ;
//      for (int i=0; i < num_buckets; i++)
//      {
//        printf("%u - %u: %u\n", (unsigned int) hist_max/num_buckets*i,
//                                (unsigned int) hist_max/num_buckets*(i+1),
//                                histogram[i]) ;
//      }
//    }
    return true;
  }
} ;

}

#endif // POINT_CLOUD_INTENSITY_FILTER_H
