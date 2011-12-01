#include <ros/ros.h>
#include <background_filters/SceneSegmentation.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <string>


typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

PointCloud::Ptr Downsample(PointCloud::Ptr cloud)
{
	PointCloud::Ptr cloud_ptr (new PointCloud); 
	double z_filter_min_ = 0.4;
	double z_filter_max_=3.25;

	//NaN filtering	
	PointCloud::Ptr cloud_filtered_ptr (new PointCloud); 
	pcl::PassThrough<Point> pass_;

	pass_.setFilterFieldName ("z");
	pass_.setFilterLimits (z_filter_min_, z_filter_max_);
	pass_.setInputCloud(cloud);
	pass_.filter (*cloud_filtered_ptr);
	ROS_INFO("NaN filtering done");
	
	//downsampling
	pcl::VoxelGrid<Point> grid_;
	PointCloud::Ptr cloud_downsampled_ptr (new PointCloud);
 	double plane_detection_voxel_size_= 0.01;

	grid_.setLeafSize (plane_detection_voxel_size_, plane_detection_voxel_size_, plane_detection_voxel_size_);
	grid_.setFilterFieldName ("z");
	grid_.setFilterLimits (z_filter_min_, z_filter_max_);
	grid_.setDownsampleAllData (false);
	grid_.setInputCloud(cloud_filtered_ptr);
	grid_.filter (*cloud_downsampled_ptr);
	ROS_INFO("Downsampling done");
	
	return cloud_downsampled_ptr;
}

void DisplayCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string title)
{
	pcl::visualization::CloudViewer viewer (title.c_str());
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ()){}
}


void DisplayCloud(PointCloud::Ptr cloud, std::string title)
{
	pcl::visualization::CloudViewer viewer (title.c_str());
	viewer.showCloud (cloud);
	while (!viewer.wasStopped ()){}
}

PointCloud::Ptr RemovePlanes(PointCloud::Ptr cloud)
{
	PointCloud::Ptr cloud_planes_removed (new PointCloud);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	

	*cloud_planes_removed = *cloud;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.03);
	seg.setMaxIterations (10000);
	seg.setProbability (0.99);
	seg.setEpsAngle(.2);

	while(true)
	{
		seg.setInputCloud (cloud_planes_removed);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () < 1000)
		{
			break;
		}

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud_planes_removed);
		extract.setIndices (inliers);
		extract.setNegative (true);
		extract.filter (*cloud_planes_removed);
	}
	return cloud_planes_removed;
}

PointCloud::Ptr DownsampleObjectsCloud(PointCloud::Ptr cloud)
{
	pcl::VoxelGrid<Point> grid_objects_;
	pcl::PointCloud<Point>::Ptr cloud_objects_downsampled_ptr (new pcl::PointCloud<Point>); 
	double clustering_voxel_size_ = 0.03;

	grid_objects_.setLeafSize (clustering_voxel_size_, clustering_voxel_size_, clustering_voxel_size_);
	grid_objects_.setDownsampleAllData (false);
	grid_objects_.setInputCloud (cloud);
	grid_objects_.filter (*cloud_objects_downsampled_ptr);
	return cloud_objects_downsampled_ptr;
}

void EuclideanSegment(PointCloud::Ptr cloud,std::vector<pcl::PointIndices>& clusters)
{
	KdTreePtr clusters_tree_;
	pcl::EuclideanClusterExtraction<Point> pcl_cluster_;
	double cluster_distance_=0.1;
	int min_cluster_size_ = 30;

	// Clustering parameters
	pcl_cluster_.setClusterTolerance (cluster_distance_);
	pcl_cluster_.setMinClusterSize (min_cluster_size_);
	clusters_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
	pcl_cluster_.setSearchMethod (clusters_tree_);

	pcl_cluster_.setInputCloud (cloud);
	pcl_cluster_.extract (clusters);
}

void DisplayClusters(PointCloud::Ptr cloud, std::vector<pcl::PointIndices>& clusters)
{
	pcl::PointCloud<pcl::PointXYZRGB> colorCloud;

	//copy original points to colorCloud
	colorCloud.width = cloud->width;
	colorCloud.height = cloud->height;
	colorCloud.points.resize(colorCloud.width*colorCloud.height);

	for(int i=0;i<cloud->points.size();i++)
	{
		colorCloud.points[i].x = cloud->points[i].x;
		colorCloud.points[i].y = cloud->points[i].y;
		colorCloud.points[i].z = cloud->points[i].z;
		colorCloud.points[i].r = 255;
		colorCloud.points[i].g = 255;
		colorCloud.points[i].b = 255;
	}

	int numColors = 7;
	int color[7][3] = {{255,0,0},{0,255,0},{0,0,255},{255,128,0},{0,128,128},{128,0,128},{255,128,128}};
	for(int i=0;i<clusters.size();i++)
	{
		
		for(int j=0;j<clusters[i].indices.size();j++)
		{
			int idx = clusters[i].indices[j];
			colorCloud.points[idx].r = color[i % numColors][0];
			colorCloud.points[idx].g = color[i % numColors][1];
			colorCloud.points[idx].b = color[i % numColors][2];
		}
	}
	
	DisplayCloud(colorCloud.makeShared(),"color cloud");

}


void 	RemovePlnaeClusters(PointCloud::Ptr cloud, std::vector<pcl::PointIndices>& clusters)
{
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	
	//PARAMS
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	seg.setMaxIterations (10000);
	seg.setProbability (0.99);

	//DETECT PLANE OBJECTS
	for(int i=0;i<clusters.size();i++)
	{
		//EXTRACT CLUSTER OUT OF THE INPUT CLOUD
		PointCloud::Ptr cloud_cluster_ptr (new PointCloud); 
		pcl::ExtractIndices<Point> extract_cluster_indices;
		extract_cluster_indices.setInputCloud (cloud);
		extract_cluster_indices.setIndices (boost::make_shared<const pcl::PointIndices> (clusters[i]));
		extract_cluster_indices.filter (*cloud_cluster_ptr);
		//SEGMENT THE CLUSTER
		seg.setInputCloud(cloud_cluster_ptr);
		seg.segment (*inliers, *coefficients);
		//TEST FOR PLANE OBJECT
		double	ratio = double(inliers->indices.size()) / clusters[i].indices.size();
		if(ratio>.7)
		{
			//ROS_INFO("plane object found:%d",i);
			clusters.erase(clusters.begin()+i);
			i--;
		}
	}
}

PointCloud::Ptr RemoveStatisticalOutliers(PointCloud::Ptr cloud)
{
	PointCloud::Ptr cloud_filtered (new PointCloud);
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.setNegative(true);
	sor.filter (*cloud_filtered);
	
	return cloud_filtered;
}

void callback3(const PointCloud::ConstPtr& point_cloud)
{
	PointCloud cloud = *point_cloud;
	ROS_INFO("input cloud size:%d",cloud.points.size());
	DisplayCloud(cloud.makeShared(),"input cloud");

	//REMOVE NaNs AND DOWNSAMPLE THE INPUT CLOUD
	PointCloud::Ptr cloud_downsampled = Downsample(cloud.makeShared());
	ROS_INFO("downsampled cloud size::%d",cloud_downsampled->points.size());
	DisplayCloud(cloud_downsampled,"downsampled cloud");

	//REMOVE PLANES
	PointCloud::Ptr cloud_planes_removed = RemovePlanes(cloud_downsampled);
	ROS_INFO("planes removed");
	//DisplayCloud(cloud_planes_removed,"planes removed cloud");


	//REMOVE STATISTICAL OUTLIERS	
	//PointCloud::Ptr cloud_outliers_removed = RemoveStatisticalOutliers(cloud_planes_removed);
	//ROS_INFO("outliers removed cloud size::%d",cloud_outliers_removed->points.size());
	//DisplayCloud(cloud_outliers_removed,"outliers removed");

	//DOWNSAMPLE OBJECTS CLOUD	
	//PointCloud::Ptr cloud_objects_downsampled = DownsampleObjectsCloud(cloud_planes_removed);
	//DisplayCloud(cloud_objects_downsampled,"objects downsampled cloud");

	//EUCLIDEAN_SEGMENT PLANE REMOVED CLOUD 
	std::vector<pcl::PointIndices> potential_objects;
	EuclideanSegment(cloud_planes_removed,potential_objects);
	ROS_INFO("clusters found:%d",potential_objects.size());
	//DisplayClusters(cloud_planes_removed,potential_objects);

	//FITER PLANE OBJECTS
	RemovePlnaeClusters(cloud_planes_removed,potential_objects);
	DisplayClusters(cloud_planes_removed,potential_objects);
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "point_cloud_bg_subtractor");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/rgb/points", 1, callback3);
  ros::spin();

  
}

