#include <ros/ros.h>
#include <point_cloud_classifier/ExtractSHOTDescriptor.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/shot.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree.h>
#include <pcl/ros/conversions.h>

using namespace pcl;

bool extract(point_cloud_classifier::ExtractSHOTDescriptor::Request  &req,
         point_cloud_classifier::ExtractSHOTDescriptor::Response &res )
{
	float		kernelSize;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::SHOTEstimation<pcl::PointXYZ,pcl::Normal,pcl::SHOT> shotExtractor;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::SHOT>::Ptr cloudShot (new pcl::PointCloud<pcl::SHOT> ());

	//initialization
	kernelSize = 5;
	sensor_msgs::PointCloud2 pcl2;
	sensor_msgs:convertPointCloudToPointCloud2(req.point_cloud,pcl2);
	fromROSMsg(pcl2,*cloudIn);
	//estimating input normals
	ne.setInputCloud (cloudIn);
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.03);// Use all neighbors in a sphere of radius 3cm
	ne.compute (*cloud_normals);
	//extract shot features for the input cloud
	shotExtractor.setInputCloud (cloudIn);
	shotExtractor.setInputNormals(cloud_normals);
	shotExtractor.setSearchMethod(tree2);
	shotExtractor.setRadiusSearch(kernelSize);
	shotExtractor.compute(*cloudShot);
	//response
	res.num_features = cloudShot->size();
	res.feature_size = cloudShot->points[0].descriptor.size();
	for(int i=0;i<cloudShot->size();i++)
	{
		for(int j=0;j<cloudShot->points[i].descriptor.size();j++)
		{
			res.shot_feature.push_back(cloudShot->points[i].descriptor[j]);
		}
	}

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "shotExtractor");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("extract_shot_descriptor", extract);
	ROS_INFO("Ready to extract shot features.");
	ros::spin();
	return 0;
}


