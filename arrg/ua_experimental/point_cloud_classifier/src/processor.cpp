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
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <point_cloud_classifier/ExtractFeatures.h>

struct PointXYZRGBIM
{
  union
  {
    struct
    {
      float x;
      float y;
      float z;
      float rgb;
      float imX;
      float imY;
    };
    float data[6];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBIM,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, rgb, rgb)
                                    (float, imX, imX)
                                    (float, imY, imY)
)


int extract_image(cv::Mat& image, cv::Mat& mask, pcl::PointCloud<PointXYZRGBIM>& cloud)
{
    int num_pixels = 0;
    
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        uint32_t rgb = *reinterpret_cast<int*>(&cloud.points[i].rgb);
        
        uint8_t r = (rgb >> 16) & 0x0000ff;
        uint8_t g = (rgb >> 8)  & 0x0000ff;
        uint8_t b = (rgb)       & 0x0000ff;
        
        int h = cloud.points[i].imY;
        int w = cloud.points[i].imX;
        
        image.at<cv::Vec3b>(h,w)[0] = b;
        image.at<cv::Vec3b>(h,w)[1] = g;
        image.at<cv::Vec3b>(h,w)[2] = r;
        
        mask.at<uint8_t>(h,w) = 255;
        ++num_pixels;
    }

    return num_pixels;
}

void calculate_hs_histogram(const cv::Mat& src, const cv::Mat& mask, cv::MatND& hist)
{
    cv::Mat hsv;
    cv::cvtColor(src, hsv, CV_BGR2HSV);

    // let's quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 30, sbins = 32;
    int histSize[] = {hbins, sbins};

    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };
    
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};

    cv::calcHist(&hsv, 1, channels, mask,       // use mask
                 hist, 2, histSize, ranges,
                 true,                          // the histogram is uniform
                 false );
    
    double maxVal = 0;
    minMaxLoc(hist, 0, &maxVal, 0, 0);

    int scale = 10;
    cv::Mat histImg = cv::Mat::zeros(sbins * scale, hbins * 10, CV_8UC3);

    for (int h = 0; h < hbins; h++)
    {
        for (int s = 0; s < sbins; s++)
        {
            float binVal = hist.at<float>(h, s);
            int intensity = cvRound(binVal * 255 / maxVal);
            cv::rectangle(histImg, cv::Point(h * scale, s * scale),
                          cv::Point( (h+1)*scale - 1, (s+1)*scale - 1),
                          cv::Scalar::all(intensity),
                          CV_FILLED);
        }
    }

    cv::namedWindow( "Source", 1 );
    imshow( "Source", src );

    cv::namedWindow( "H-S Histogram", 1 );
    imshow( "H-S Histogram", histImg );
}

void calculate_rgb_histogram(const cv::Mat& src, const cv::Mat& mask, int num_pixels, cv::MatND& hist)
{
    int bins = 8;
    int histSize[] = { bins, bins, bins };
    float vranges[] = { 0, 256 };
    const float* ranges[] = { vranges, vranges, vranges };
    int channels[] = {0, 1, 2};

    cv::calcHist(&src, 1, channels, mask,       // use mask
                 hist, 3, histSize, ranges,
                 true,                          // the histogram is uniform
                 false);
    
    hist /= num_pixels;
    
    for (int b = 0; b < bins; ++b)
    {
        for (int g = 0; g < bins; ++g)
        {
            for (int r = 0; r < bins; ++r)
            {
                float binVal = hist.at<float>(b, g, r);
                if (binVal != 0) { ROS_INFO("%d %d %d = %f", b, g, r, binVal); }
            }
        }
    }
}

void extract_features(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::SHOT>& shot_features)
{
    float kernelSize = 5;

    pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT> shotExtractor;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree2(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // estimating input normals
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    
    // use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_normals);
    
    //extract shot features for the input cloud
    shotExtractor.setInputCloud(cloud);
    shotExtractor.setInputNormals(cloud_normals);
    shotExtractor.setSearchMethod(tree2);
    shotExtractor.setRadiusSearch(kernelSize);
    shotExtractor.compute(shot_features);
}

template <typename PointInT, typename PointOutT> void
copyPointCloudCustom (const pcl::PointCloud<PointInT> &cloud_in, pcl::PointCloud<PointOutT> &cloud_out)
{
    // Allocate enough space and copy the basics
    cloud_out.points.resize (cloud_in.points.size ());
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = cloud_in.points.size ();
    cloud_out.height   = 1;
    cloud_out.is_dense = cloud_in.is_dense;
 
    // Copy all the data fields from the input cloud to the output one
    typedef typename pcl::traits::fieldList<PointOutT>::type FieldList;
    
    // Iterate over each point
    for (size_t i = 0; i < cloud_in.points.size (); ++i)
    {
        // Iterate over each dimension
        pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <PointInT, PointOutT> (cloud_in.points[i], cloud_out.points[i]));
    }
}

bool extract(point_cloud_classifier::ExtractFeatures::Request  &req,
             point_cloud_classifier::ExtractFeatures::Response &res)
{
    //std::string fname("/home/anton/Downloads/rgbd-dataset/apple/apple_1/apple_1_1_1.pcd");
    std::string fname = req.pcd_file_name;
    
    pcl::PointCloud<PointXYZRGBIM>::Ptr cloud(new pcl::PointCloud<PointXYZRGBIM>);
    
    if (pcl::io::loadPCDFile<PointXYZRGBIM>(fname, *cloud) == -1)
    {
        ROS_ERROR("Could not read file %s", fname.c_str());
        return false;
    }
    
    ROS_INFO("Loaded %d data points from %s", cloud->width * cloud->height, fname.c_str());
    ROS_INFO("\tCloud size is (%d,%d)", cloud->width, cloud->height);
    
    cv::Mat image(480, 640, CV_8UC3);
    cv::Mat mask = cv::Mat::zeros(480, 640, CV_8UC1);
    int num_pixels = extract_image(image, mask, *cloud);
    
    //cv::namedWindow("apple");
    //cv::imshow("apple", image);
    
    //cv::namedWindow("mask");
    //cv::imshow("mask", mask);
    
    //cv::waitKey(0);
    
    ROS_INFO("\tCopying clouds...");
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr shot_in(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloudCustom<PointXYZRGBIM, pcl::PointXYZ>(*cloud, *shot_in);
    
    ROS_INFO("\tCalculating SHOT descriptor...");
    
    pcl::PointCloud<pcl::SHOT>::Ptr cloud_shot(new pcl::PointCloud<pcl::SHOT>);
    extract_features(shot_in, *cloud_shot);
    
    int num_features = cloud_shot->size();
    int feature_size = cloud_shot->points[0].descriptor.size();

    ROS_INFO("\tNumber of SHOT features: %d, feature size: %d", num_features, feature_size);
    
    cv::MatND hist;
    calculate_rgb_histogram(image, mask, num_pixels, hist);
    
    //response
    res.num_features = num_features;
    res.feature_size = feature_size;
    
    for (int i = 0; i < cloud_shot->size(); ++i)
    {
        for(int j = 0; j < cloud_shot->points[i].descriptor.size(); ++j)
        {
            res.shot_feature.push_back(cloud_shot->points[i].descriptor[j]);
        }
    }
    
    res.num_bins = 8;
    
    for (int b = 0; b < res.num_bins; ++b)
    {
        for (int g = 0; g < res.num_bins; ++g)
        {
            for (int r = 0; r < res.num_bins; ++r)
            {
                res.color_hist.push_back(hist.at<float>(b, g, r));
            }
        }
    }

    return true;

}


int main (int argc, char* argv[])
{
    ros::init(argc, argv, "pointcloud_processor");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("extract_features", extract);
    
    ros::spin();
}
