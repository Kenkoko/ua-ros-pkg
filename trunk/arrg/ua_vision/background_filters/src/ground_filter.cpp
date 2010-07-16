#include <iostream>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_types.h>

#include <pcl/ros/subscriber.h>
#include <pcl/ros/publisher.h>

#include <pcl/io/pcd_io.h>

#include <pcl/features/normal_3d.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/surface/convex_hull.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

using namespace std;
using namespace pcl;

typedef PointXYZRGB PointT;
typedef PointCloud<PointT> CloudT;

point_cloud::Publisher<PointXYZRGB> cluster_pub;
point_cloud::Publisher<PointXYZRGB> objects_pub;
point_cloud::Publisher<PointXYZRGB> hull_pub;
ros::Publisher marker_pub;

// Returns the points that are above the floor, but not the floor itself
CloudT get_object_points(CloudT cloud)
{
  // PHASE 1: DATA CLEANUP
  ////////////////////////////////////////////////////////

  // Filter out NaNs from data (this is necessary now) ...
  PassThrough<PointXYZRGB> nan_remover;
  nan_remover.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(cloud));
  nan_remover.setFilterFieldName("z");
  nan_remover.setFilterLimits(0.0, 10.0);
  nan_remover.filter(cloud);

  // Filter out statistical outliers
  StatisticalOutlierRemoval<PointXYZRGB> statistical_filter;
  statistical_filter.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(cloud));
  statistical_filter.setMeanK(50);
  statistical_filter.setStddevMulThresh(1.0);
  statistical_filter.filter(cloud);

  // Downsample to 1cm Voxel Grid
  pcl::VoxelGrid<PointT> downsampler;
  downsampler.setInputCloud(boost::make_shared<CloudT>(cloud));
  downsampler.setLeafSize(0.005, 0.005, 0.005);
  downsampler.filter(cloud);
  ROS_INFO("PointCloud after filtering: %d data points.", cloud.width * cloud.height);

  // PHASE 2: FIND THE GROUND PLANE
  /////////////////////////////////////////////////////////

  // Step 3: Find the ground plane using RANSAC
  ModelCoefficients ground_coefficients;
  PointIndices ground_indices;
  SACSegmentation<PointXYZRGB> ground_finder;
  ground_finder.setOptimizeCoefficients(true);
  ground_finder.setModelType(SACMODEL_PLANE);
  ground_finder.setMethodType(SAC_RANSAC);
  ground_finder.setDistanceThreshold(0.015);
  ground_finder.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(cloud));
  ground_finder.segment(ground_indices, ground_coefficients);

  // PHASE 3: EXTRACT ONLY POINTS ABOVE THE GROUND PLANE
  /////////////////////////////////////////////////////////

  // Step 3a. Extract the ground plane inliers
  CloudT ground_points;
  ExtractIndices<PointXYZRGB> extractor;
  extractor.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(cloud));
  extractor.setIndices(boost::make_shared<PointIndices>(ground_indices));
  extractor.filter(ground_points);

  // Step 3b. Extract the ground plane outliers
  CloudT object_points;
  ExtractIndices<PointXYZRGB> outlier_extractor;
  outlier_extractor.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(cloud));
  outlier_extractor.setIndices(boost::make_shared<PointIndices>(ground_indices));
  outlier_extractor.setNegative(true);
  outlier_extractor.filter(object_points);

  // Step 3c. Project the ground inliers
  PointCloud<PointXYZRGB> cloud_projected;
  ProjectInliers<PointXYZRGB> proj;
  proj.setModelType(SACMODEL_PLANE);
  proj.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(ground_points));
  proj.setModelCoefficients(boost::make_shared<ModelCoefficients>(ground_coefficients));
  proj.filter(cloud_projected);

  // Step 3d. Create a Convex Hull representation of the projected inliers
  PointCloud<PointXYZRGB> ground_hull;
  ConvexHull2D<PointXYZRGB, PointXYZRGB> chull;
  chull.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(cloud_projected));
  chull.reconstruct(ground_hull);

  ROS_INFO ("Convex hull has: %d data points.", (int) ground_hull.points.size ());

  hull_pub.publish(ground_hull);

  // Step 3e. Extract only those outliers that lie above the ground plane's convex hull
  PointIndices object_indices;
  ExtractPolygonalPrismData<PointT> hull_limiter;
  hull_limiter.setInputCloud(boost::make_shared<CloudT>(object_points));
  hull_limiter.setInputPlanarHull(boost::make_shared<CloudT>(ground_hull));
  hull_limiter.setHeightLimits(0, 0.3);
  hull_limiter.segment(object_indices);

  ExtractIndices<PointT> object_extractor;
  object_extractor.setInputCloud(boost::make_shared<CloudT>(object_points));
  object_extractor.setIndices(boost::make_shared<PointIndices>(object_indices));
  object_extractor.filter(object_points);

  return object_points;
}

void segment(PointCloud<PointXYZRGB> cloud)
{
  CloudT object_cloud = get_object_points(cloud);

  objects_pub.publish(object_cloud);

  // Estimate point normals
  //  PointCloud<Normal> cloud_normals;
  //  KdTreeANN<PointXYZRGB>::Ptr tree = boost::make_shared<KdTreeANN<PointXYZRGB> >();
  //  NormalEstimation<PointXYZRGB, Normal> ne;
  //  ne.setSearchMethod(tree);
  //  ne.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(object_cloud));
  //  ne.setKSearch(50);
  //  ne.compute(cloud_normals);

  // Extract clusters of points (i.e., objects)
  EuclideanClusterExtraction<PointXYZRGB> clustering;
  KdTreeANN<PointXYZRGB>::Ptr cluster_tree = boost::make_shared<KdTreeANN<PointXYZRGB> >();
  vector<PointIndices> clusters;
  clustering.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(object_cloud));
  clustering.setClusterTolerance(0.05); // ?
  clustering.setMinClusterSize(300);
  clustering.setSearchMethod(cluster_tree); // Not sure if this should be ANN, or FLANN, or if it matters
  clustering.extract(clusters);

  //  cout << (cloud_normals.height * cloud_normals.width) << " " << (object_cloud.height * object_cloud.width) << endl;

  cout << "FOUND " << clusters.size() << " OBJECTS" << endl;
  for (uint i = 0; i < clusters.size(); i++)
  {
    cout << "CLUSTER " << i << " has " << clusters[i].indices.size() << " points" << endl;

    PointCloud<PointXYZRGB> cluster_points;
    ExtractIndices<PointXYZRGB> extract_cluster;
    extract_cluster.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(object_cloud));
    extract_cluster.setIndices(boost::make_shared<PointIndices>(clusters[i]));
    extract_cluster.filter(cluster_points);

    cluster_pub.publish(cluster_points);

    // Estimate point normals
    PointCloud<Normal> cluster_normals;
    KdTreeANN<PointT>::Ptr tree = boost::make_shared<KdTreeANN<PointT> >();
    // Estimate point normals
    NormalEstimation<PointT, Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(cluster_points));
    ne.setKSearch(10);
    ne.compute(cluster_normals);

    //    NormalEstimation<PointT, Normal> ne;
    //    ne.setSearchMethod(tree);
    //    ne.setInputCloud(boost::make_shared<CloudT>(cluster_points));
    //    ne.setKSearch(50);
    //    ne.compute(cluster_normals);

    //    cout << cluster_points.height * cluster_points.width << " " << cluster_normals.height * cluster_normals.width
    //        << endl;

    // Obtain the plane inliers and coefficients
    ModelCoefficients coefficients_plane;
    PointIndices inliers_plane;
    SACSegmentationFromNormals<PointT, Normal> seg_plane;
    seg_plane.setOptimizeCoefficients(true);
    seg_plane.setMethodType(SAC_RANSAC);
    seg_plane.setModelType(SACMODEL_NORMAL_PLANE);
    seg_plane.setDistanceThreshold(0.05);
    seg_plane.setInputCloud(boost::make_shared<CloudT>(cluster_points));
    seg_plane.setInputNormals(boost::make_shared<PointCloud<Normal> >(cluster_normals));
    seg_plane.segment(inliers_plane, coefficients_plane);

    //    cout << "Plane coefficients: " << coefficients_plane << endl;
    cout << "PLANE INLIERS: " << inliers_plane.indices.size() << endl;

    // Obtain the Sphere inliers and coefficients
    ModelCoefficients coefficients_sphere;
    PointIndices inliers_sphere;
    SACSegmentation<PointXYZRGB> seg_sphere;
    seg_sphere.setOptimizeCoefficients(true);
    seg_sphere.setMethodType(SAC_RANSAC);
    seg_sphere.setModelType(SACMODEL_SPHERE);
    seg_sphere.setRadiusLimits(0.01, 0.1);
    seg_sphere.setDistanceThreshold(0.005);
    seg_sphere.setInputCloud(boost::make_shared<CloudT>(cluster_points));
    seg_sphere.segment(inliers_sphere, coefficients_sphere);

    //    cout << "Sphere coefficients: " << coefficients_sphere << endl;
    cout << "SPHERE INLIERS: " << inliers_sphere.indices.size() << endl;

    ModelCoefficients coefficients_cylinder;
    PointIndices inliers_cylinder;
    SACSegmentationFromNormals<PointT, Normal> seg_cylinder;
    seg_cylinder.setOptimizeCoefficients(true);
    seg_cylinder.setModelType(SACMODEL_CYLINDER);
    seg_cylinder.setMethodType(SAC_RANSAC);
    seg_cylinder.setNormalDistanceWeight(0.1);
    seg_cylinder.setMaxIterations(1000);
    seg_cylinder.setDistanceThreshold(0.05);
    seg_cylinder.setRadiusLimits(0.01, 0.1);
    seg_cylinder.setInputCloud(boost::make_shared<CloudT>(cluster_points));
    seg_cylinder.setInputNormals(boost::make_shared<PointCloud<Normal> >(cluster_normals));
    seg_cylinder.segment(inliers_cylinder, coefficients_cylinder);

    cout << "CYLINDER INLIERS: " << inliers_cylinder.indices.size() << endl;
    cout << "Cylinder coefficients: " << coefficients_cylinder << endl;

    cout << endl;

    if (!inliers_sphere.indices.empty())
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = coefficients_sphere.header.frame_id;
      marker.header.stamp = ros::Time::now();
      marker.ns = "basic_shapes" + boost::lexical_cast<string>(i);
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = coefficients_sphere.values[0];
      marker.pose.position.y = coefficients_sphere.values[1];
      marker.pose.position.z = coefficients_sphere.values[2];
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = coefficients_sphere.values[3] * 2;
      marker.scale.y = coefficients_sphere.values[3] * 2;
      marker.scale.z = coefficients_sphere.values[3] * 2;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration();
      marker_pub.publish(marker);
    }

    if (!inliers_cylinder.indices.empty())
    {
      // TODO: The cylinder coefficients are not as straightforward, need to do some
      // math to pull out the location and angle (from axis orientation and point on axis)
//      visualization_msgs::Marker marker;
//      marker.header.frame_id = coefficients_sphere.header.frame_id;
//      marker.header.stamp = ros::Time::now();
//      marker.ns = "basic_shapes" + boost::lexical_cast<string>(i);
//      marker.id = 0;
//      marker.type = visualization_msgs::Marker::SPHERE;
//      marker.action = visualization_msgs::Marker::ADD;
//      marker.pose.position.x = coefficients_sphere.values[0];
//      marker.pose.position.y = coefficients_sphere.values[1];
//      marker.pose.position.z = coefficients_sphere.values[2];
//      marker.pose.orientation.x = 0.0;
//      marker.pose.orientation.y = 0.0;
//      marker.pose.orientation.z = 0.0;
//      marker.pose.orientation.w = 1.0;
//      marker.scale.x = coefficients_sphere.values[3] * 2;
//      marker.scale.y = coefficients_sphere.values[3] * 2;
//      marker.scale.z = coefficients_sphere.values[3] * 2;
//      marker.color.r = 0.0f;
//      marker.color.g = 1.0f;
//      marker.color.b = 0.0f;
//      marker.color.a = 1.0;
//      marker.lifetime = ros::Duration();
//      marker_pub.publish(marker);
    }
  }
}

bool die = false;
void callback(const PointCloud<PointXYZRGB>::ConstPtr& msg)
{
  if (!die)
  {
    segment(*msg);
    //    die = true;
  }
}

int main(int argc, char** argv)
{
  // Standard node stuff
  ros::init(argc, argv, "point_cloud_listener");
  ros::NodeHandle nh;

  // CAPTURE

  cluster_pub.advertise(nh, "cluster_points", 1);
  objects_pub.advertise(nh, "object_points", 1);
  hull_pub.advertise(nh, "hull_points", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker> ("visualization_marker", 0);

  point_cloud::Subscriber<PointXYZRGB> sub;
  sub.subscribe(nh, "/stereo/points2", 1, callback);

  ros::spin();

  return 0;

}

// Step 2: Downsample the point cloud (to save time in the next step)
//  VoxelGrid<PointXYZRGB> downsampler;
//  downsampler.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(cloud_filtered));
//  downsampler.setLeafSize(0.01, 0.01, 0.01); // leaf size of 1cm
//  downsampler.filter(cloud_filtered);
//  ROS_INFO("After downsample: %d", cloud_filtered.width * cloud_filtered.height);


