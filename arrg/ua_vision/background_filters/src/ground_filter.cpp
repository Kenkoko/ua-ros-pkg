#include <iostream>
#include <boost/foreach.hpp>

#include <ros/ros.h>

#include <pcl/point_types.h>

#include <pcl/ros/subscriber.h>
#include <pcl/ros/publisher.h>

#include <pcl/io/pcd_io.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

point_cloud::Publisher<pcl::PointXYZRGB> pub;
point_cloud::Publisher<pcl::PointXYZRGB> pub2;

void segment(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB> ground_points;
  pcl::PointCloud<pcl::PointXYZRGB> object_points;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

  std::cout << "Total points: " << cloud.points.size() << std::endl;

  // Step 1: Filter out statistical outliers
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud));
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(cloud_filtered);

  // Step 2: Downsample the point cloud (to save time in the next step)
  pcl::VoxelGrid<pcl::PointXYZRGB> downsampler;
  downsampler.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud_filtered));
  downsampler.setLeafSize(0.01, 0.01, 0.01); // leaf size of 1cm
  downsampler.filter(cloud_filtered);

  ROS_INFO("After filtering: %d", cloud_filtered.width * cloud_filtered.height);

  // Step 3: Find the ground plane using RANSAC
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true); // Optional
//  seg.setMaxIterations(500); // Optional, maybe can be lower
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02);
  seg.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud_filtered));
  seg.segment(inliers, coefficients);

  if (inliers.indices.size() == 0)
  {
    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
  }

  std::cout << "Model inliers: " << inliers.indices.size() << std::endl;

  // Extract the points that lie in the ground plane
  pcl::ExtractIndices<pcl::PointXYZRGB> extractor;
  extractor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud_filtered));
  extractor.setIndices(boost::make_shared<pcl::PointIndices>(inliers));
  extractor.filter(ground_points);

  // Extract the points that are objects (i.e., are not in the ground plane)
  extractor.setNegative(true);
  extractor.filter(object_points);

  pub.publish(ground_points);
  pub2.publish(object_points);
}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
{
  //  printf("Cloud: width = %u, height = %u\n", msg->width, msg->height);
  //BOOST_FOREACH (const pcl::PointXYZRGB pt, msg->points)
  //  printf("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

  // Save the point cloud to file
  //pcl::io::savePCDFile("test.pcd", *msg, false);
  //ros::shutdown();

  segment(*msg);
}

int main(int argc, char** argv)
{
  // Standard node stuff
  ros::init(argc, argv, "point_cloud_listener");
  ros::NodeHandle nh;

  // CAPTURE

  pub.advertise(nh, "floor_points", 1);
  pub2.advertise(nh, "object_points", 1);

  point_cloud::Subscriber<pcl::PointXYZRGB> sub;
  sub.subscribe(nh, "/stereo/points2", 1, callback);

  ros::spin();

  // PROCESS
  //  sensor_msgs::PointCloud2 cloud_blob;
  //  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  //
  //  if (pcl::io::loadPCDFile("test.pcd", cloud_blob) == -1)
  //  {
  //    ROS_ERROR ("Couldn't read file test_pcd.pcd");
  //    return (-1);
  //  }
  //  ROS_INFO ("Loaded %d data points from test_pcd.pcd with the following fields: %s", (int)(cloud_blob.width * cloud_blob.height), pcl::getFieldsList (cloud_blob).c_str ());
  //
  //  // Convert to the templated message type
  //  point_cloud::fromMsg(cloud_blob, cloud);
  //
  //  segment(cloud);

  return 0;

}
