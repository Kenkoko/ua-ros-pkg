#include <ros/ros.h>
#include <tf/tf.h>

// Messages
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>


// Drivers used
#include <stoc.h>

#include <sstream>

////////////////////////////////////////////////////////////////////////////////
// Start
int start (stoc::STOC &stoc)
{
    // Open the STOC device
    try
    {
        if (stoc.open () == 0)
          ROS_INFO ("[STOC Publisher] Connected to device with ID: %s", stoc.device_id_.c_str ());
        } catch (stoc::Exception& e) {
        ROS_ERROR("[STOC Publisher] Exception thrown while connecting to the sensor.\n%s", e.what ());
    }

    return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Stop
int stop (stoc::STOC &stoc)
{
    // Close the STOC device
    if (stoc.close () == 0)
        ROS_INFO ("[STOC Publisher] Driver shut down successfully");
    else
        return (-1);

    return (0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stoc_publisher");
    ros::NodeHandle n;
    ros::Publisher left_pub = n.advertise<sensor_msgs::Image>("stereo/left/image_raw", 30);
    ros::Publisher right_pub = n.advertise<sensor_msgs::Image>("stereo/right/image_raw", 30);
    ros::Publisher disparity_pub = n.advertise<stereo_msgs::DisparityImage>("stereo/image_disparity", 30);
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("stereo/cloud", 30);

    ros::Rate loop_rate(30);

    stoc::STOC stoc_;
    start(stoc_);

    sensor_msgs::PointCloud point_cloud;

    sensor_msgs::Image left_image;
    sensor_msgs::Image right_image;
    stereo_msgs::DisparityImage disparity_image;

    int count = 0;
    while (ros::ok())
    {
        try
        {
            // Read data from the STOC
            stoc_.readData(point_cloud, left_image, right_image, disparity_image);
        } catch (stoc::Exception& e) {
            ROS_WARN("[STOC Publisher] Exception thrown while trying to read data.\n%s", e.what ());
            continue;
        }

        // Publish it
        left_pub.publish(left_image);
        right_pub.publish(right_image);
        disparity_pub.publish(disparity_image);
        cloud_pub.publish(point_cloud);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    stop(stoc_);    
}

