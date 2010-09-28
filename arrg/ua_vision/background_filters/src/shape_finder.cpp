#include <iostream>
#include <utility>
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
#include <tf_pcl/transforms.h>

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

// camera info...these constants are used throughout this whole file
const double camera_roll = 2.4;
const double camera_height = 0.53;

// returns a transformation (rotation and translation) from camera to world -- i.e.,
// if x_c is a point in camera corrdinates and <R, t> = cam_to_world_trans(), then
// x_w = R*x_c + t is that same point in world coordinates.
pair<btMatrix3x3, btVector3> cam_to_world_trans()
{
    btMatrix3x3 rotation(btQuaternion(0.0, 0.0, -camera_roll));
    btVector3 translation(0, 0, camera_height);

    return make_pair(rotation, translation);
}

// returns a transformation (rotation and translation) from world to camera -- i.e.,
// if x_w is a point in world corrdinates and <R, t> = world_to_cam_trans(), then
// x_c = R*x_w + t is that same point in camera coordinates.
pair<btMatrix3x3, btVector3> world_to_cam_trans()
{
    btMatrix3x3 rotation(btQuaternion(0.0, 0.0, camera_roll));
    btVector3 translation(0, 0, -camera_height);

    return make_pair(rotation, rotation * translation);
}

// display cube or sphere (in world coordinates) in rviz (in camera coordinates)
void display_shape(VectorXd& params, ModelCoefficients& coefficients_plane, uint32_t shape)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = coefficients_plane.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    if(marker.type == visualization_msgs::Marker::CUBE)
    {
        double width = params[0];
        double length = params[1];
        double height = params[2];
        double x_loc = params[3];
        double y_loc = params[4];
        double angle = params[5];

        pair<btMatrix3x3, btVector3> w2c = world_to_cam_trans();

        btVector3 cube_center_w(x_loc, y_loc, height / 2.0);
        btVector3 cube_center_c = w2c.first * cube_center_w + w2c.second;

        btMatrix3x3 cube_rotation_w(btQuaternion(angle, 0.0, 0.0));
        btMatrix3x3 cube_rotation_c = w2c.first * cube_rotation_w;
        btQuaternion cube_rotation_c_q;
        cube_rotation_c.getRotation(cube_rotation_c_q);

        marker.pose.position.x = cube_center_c.x();
        marker.pose.position.y = cube_center_c.y(); 
        marker.pose.position.z = cube_center_c.z();

        marker.pose.orientation.x = cube_rotation_c_q.getX();
        marker.pose.orientation.y = cube_rotation_c_q.getY();
        marker.pose.orientation.z = cube_rotation_c_q.getZ();
        marker.pose.orientation.w = cube_rotation_c_q.getW();

        marker.scale.x = width;
        marker.scale.y = length;
        marker.scale.z = height;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.9;
    }
    else
    {
        double x_loc = params[0];
        double y_loc = params[1];
        double radius = params[2];

        pair<btMatrix3x3, btVector3> w2c = world_to_cam_trans();

        btVector3 sphere_center_w(x_loc, y_loc, radius);
        btVector3 sphere_center_c = w2c.first * sphere_center_w + w2c.second;

        marker.pose.position.x = sphere_center_c.x();
        marker.pose.position.y = sphere_center_c.y(); 
        marker.pose.position.z = sphere_center_c.z();

        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.scale.z = radius;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.9;
    }

    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
}

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

pair<double, uint32_t> kjb_fit_shapes(const PointCloud<PointXYZRGBNormal>& data, unsigned int i, VectorXd& params)
{
    ofstream out;
    stringstream ss(stringstream::in | stringstream::out);
    ss << "cluster_" << i;

    out.open(ss.str().c_str());
    if(out.fail())
    {
        throw runtime_error("Could not open data file " + ss.str());
    }

    for(unsigned int n = 0; n < data.points.size(); n++)
    {
        out << data.points[n].x << " " << data.points[n].y << " " << data.points[n].z << " ";
        out << data.points[n].normal[0] << " " << data.points[n].normal[1] << " " << data.points[n].normal[2] << endl;
    }
    out.close();

    if(out.fail())
    {
        throw runtime_error("Could not close data file " + ss.str());
    }

    stringstream ss2(stringstream::in | stringstream::out);
    string res_file_name = ss.str() + "_results";
    ss2 << path_to_kjb_shape_fitting.c_str() << " " << ss.str() << " " << res_file_name;

    int result = system(ss2.str().c_str());
    //cout << "Result is" << result << endl;

    if(result)
    {
        throw runtime_error("Could not execute " + path_to_kjb_shape_fitting);
    }

    ifstream ifs;
    ifs.open(res_file_name.c_str());

    if(ifs.fail())
    {
        throw runtime_error("Could not open input file " + res_file_name);
    }

    VectorXd cuboid_params(6);
    double cuboid_probability;
    ifs >> cuboid_params[0] >> cuboid_params[1] >> cuboid_params[2] >> cuboid_params[3] >> cuboid_params[4] >> cuboid_params[5] >> cuboid_probability;
    if(ifs.fail())
    {
        throw runtime_error("Could not read from input file " + res_file_name);
    }

    VectorXd sphere_params(6);
    double sphere_probability;
    ifs >> sphere_params[0] >> sphere_params[1] >> sphere_params[2] >> sphere_probability;
    if(ifs.fail())
    {
        throw runtime_error("Could not read from input file " + res_file_name);
    }

    ifs.close();
    if(ifs.fail())
    {
        throw runtime_error("Could not close input file " + res_file_name);
    }

    //Display the cubes
    if(cuboid_probability > sphere_probability)
    {
        params = cuboid_params;
        return make_pair(cuboid_probability, static_cast<unsigned int>(visualization_msgs::Marker::CUBE));
    }

    //else
    params = sphere_params;
    return make_pair(cuboid_probability, static_cast<unsigned int>(visualization_msgs::Marker::SPHERE));
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
        cout << "\tCLUSTER " << i << " has " << clusters[i].indices.size() << " points" << endl;

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

        if(!inliers_plane.indices.empty())
        {
            cout << "\t\tNumber of object cloud points is: " << object_cloud.points.size() << endl;
            cout << "\t\tNumber of cluster points is: " << cluster_points.points.size() << endl; 
            cout << "\t\tNumber of cluster normals is: " << cluster_normals.points.size() << endl;

            pair<btMatrix3x3, btVector3> c2w = cam_to_world_trans();
            PointCloud<PointXYZRGBNormal> points_normals_c;
            PointCloud<PointXYZRGBNormal> points_normals_w;
            pcl::concatenateFields(cluster_points, cluster_normals, points_normals_c);
            const tf::Transform tr = tf::Transform(c2w.first, c2w.second);
            pcl::transformPointCloudWithNormals(points_normals_c, points_normals_w, tr);

            VectorXd params;
            pair<double, uint32_t> results = kjb_fit_shapes(points_normals_w, i, params);

            display_shape(params, coefficients_plane, results.second);
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
    if(argc !=2)
    {
        cout << "Error, path to kjb_shape_fitter should be provided in input" << endl;
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

