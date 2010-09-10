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

// For GJK detection 
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btBox2dShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btConvexPointCloudShape.h>

#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

// For distributions
#include <boost/random.hpp>
#include <boost/math/distributions/uniform.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/laplace.hpp>

#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include <string>


// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN


using namespace std;
using namespace pcl;

typedef PointXYZRGB PointT;
typedef PointCloud<PointT> CloudT;

point_cloud::Publisher<PointXYZRGB> cluster_pub;
point_cloud::Publisher<PointXYZRGB> objects_pub;
point_cloud::Publisher<PointXYZRGB> hull_pub;
ros::Publisher marker_pub;

string path_to_kjb_shape_fitting;

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

bool kjb_fit_shapes( const PointCloud<PointXYZRGB> & cluster_points, const PointCloud<Normal> & cluster_normals, 
                     VectorXf & cuboid_params, double & cuboid_probability,
                     VectorXf & sphere_params, double & sphere_probability,
                     uint i)
{
     using namespace std;
    
    ofstream out;
    stringstream ss(stringstream::in | stringstream::out);
    ss << "Cluster_" << i;
    std::cout << ss.str().c_str() << std::endl;
    out.open(ss.str().c_str());
    if(out.fail())
    {
        std::cout << "COULD NOT OPEN OUTPUT FILE" << ss.str().c_str() << std::endl;
        return false;
    }

    for(uint n = 0; n <cluster_points.points.size(); n++)
    {
        out << cluster_points.points[n].x << " " << cluster_points.points[n].y << " " << cluster_points.points[n].z << " ";
        out << cluster_normals.points[n].normal[0] << " " << cluster_normals.points[n].normal[1] << " " << cluster_normals.points[n].normal[2] << std::endl;
    }

    out.close();
    if(out.fail())
    {
        std::cout << "COULD NOT CLOSE OUTPUT FILE" << ss.str().c_str() << std::endl;
        return false;
    }
    
    stringstream ss2(stringstream::in | stringstream::out);
    ss2 << path_to_kjb_shape_fitting.c_str() << " " << ss.str() << " " << ss.str() << "_o"; 
    
    int result = system(ss2.str().c_str());
    std::cout << "Result is" << result << std::endl;
    if(result)
    {
        std::cout << "Could not execute kjb_fit_shapes" << std::endl;
        return false;
    }

    ifstream ifs;
    ss << "_o";
    ifs.open(ss.str().c_str());

    if(ifs.fail())
    {
        std::cout << "COULD NOT OPEN INPUT FILE" << ss.str().c_str() << std::endl;
        return false;
    }

    ifs >> cuboid_params[0] >> cuboid_params[1] >> cuboid_params[2] >> cuboid_params[3] >> cuboid_params[4] >> cuboid_params[5] >> cuboid_probability;

    if(ifs.fail())
    {
        std::cout << "COULD NOT READ CUBOID FROM INPUT FILE" << ss.str().c_str() << std::endl;
        return false;
    }

    ifs >> sphere_params[0] >> sphere_params[1] >> sphere_params[2] >>  (sphere_probability);

    if(ifs.fail())
    {
        std::cout << "COULD NOT READ SPHERE FROM INPUT FILE" << ss.str().c_str() << std::endl;
        return false;
    }
    ifs.close();
    if(ifs.fail())
    {
        std::cout << "COULD NOT CLOSE INPUT FILE" << ss.str().c_str() << std::endl;
        return false;
    }

    std::cout << "Cuboid:" << std::endl;
    std::cout <<  cuboid_params[0] << " " <<  cuboid_params[1] << " " <<  cuboid_params[2] << " " <<
                  cuboid_params[3] << " " <<  cuboid_params[4] << " " << cuboid_params[5]  << " " << cuboid_probability << std::endl;
    std::cout << "Sphere:" << std::endl;
    std::cout << sphere_params[0] << " " << sphere_params[1] << " " <<  sphere_params[2] << " " << sphere_probability << std::endl; 
     
    return true;
} 

void segment(PointCloud<PointXYZRGB> cloud)
{
  CloudT object_cloud = get_object_points(cloud);

  objects_pub.publish(object_cloud);

  // Extract clusters of points (i.e., objects)
  EuclideanClusterExtraction<PointXYZRGB> clustering;
  KdTreeANN<PointXYZRGB>::Ptr cluster_tree = boost::make_shared<KdTreeANN<PointXYZRGB> >();
  vector<PointIndices> clusters;
  clustering.setInputCloud(boost::make_shared<PointCloud<PointXYZRGB> >(object_cloud));
  clustering.setClusterTolerance(0.05); 
  clustering.setMinClusterSize(300);
  clustering.setSearchMethod(cluster_tree); // Not sure if this should be ANN, or FLANN, or if it matters
  clustering.extract(clusters);

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
    ne.setKSearch(20);
    ne.compute(cluster_normals);

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

    if (!inliers_plane.indices.empty())
    {
      cout<<"Number of object cloud is: " <<object_cloud.points.size()<<endl;
      cout<<"Number of cluster_points is: " <<cluster_points.points.size()<<endl; 
      cout<<"Number of cluster_normals is: "<<cluster_normals.points.size()<<endl;
      
      VectorXf cuboid_params(6);
      double cuboid_probability;
      VectorXf sphere_params(3);
      double sphere_probability;
      kjb_fit_shapes(cluster_points, cluster_normals,cuboid_params, cuboid_probability, sphere_params, sphere_probability, i);
    }
    
  }
}

bool die = false;
void callback(const PointCloud<PointXYZRGB>::ConstPtr& msg)
{
  if (!die)
  {
    segment(*msg);
  }
}

int main(int argc, char** argv)
{
  
  if(argc !=2 )
  {
      std::cout << "Error, path to kjb_shape_fitter should be provided in input" << std::endl;
      return 1;
  }
  path_to_kjb_shape_fitting.clear();
  path_to_kjb_shape_fitting.append(argv[1]);

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
