#include <ros/ros.h>
#include <sstream>
#include "background_filters/floor_filter.h"
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

// TODO: Document this class
// TODO: Make it (and FloorFilter) parameterizeable (frame and threshold/dimension, etc.)
// TODO: Implement some kind of smoothing
    
class FloorFilterNode
{

public:

    FloorFilterNode(void):  nh_("~"), 
                            sub_(root_handle_, "cloud_in", 1),
                            tf_filter_(sub_, tf_, "/map", 10)
    {
        floor_filter_ = new filters::FloorFilter();

        pointCloudPublisher_ = root_handle_.advertise<sensor_msgs::PointCloud>("cloud_out", 1);
      
        tf_filter_.registerCallback(boost::bind(&FloorFilterNode::filterCallback, this, _1));
    }

    ~FloorFilterNode(void)
    {
        delete floor_filter_;
    }
    
private:
    
    // This method only gets called when the transform is ready
    void filterCallback(const sensor_msgs::PointCloudConstPtr &cloud_in) {
        sensor_msgs::PointCloud cloud_base, cloud_out;
        const std::string target_frame("/map");

        try {
                
            // Use TF to transform the point cloud into global frame
            tf_.transformPointCloud(target_frame, *cloud_in, cloud_base);

            // The time when we started filtering
            ros::WallTime tm = ros::WallTime::now();
            
            // Filter out points on the floor
            floor_filter_->update(cloud_base, cloud_out);

            // The time when we finished filtering
            double sec = (ros::WallTime::now() - tm).toSec();

            // Publish the resulting point cloud
            pointCloudPublisher_.publish(cloud_out);    

            ROS_DEBUG("FloorFilter: reduced %d points to %d points in %f seconds", (int)cloud_base.points.size(), (int)cloud_out.points.size(), sec);

        } catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());                                
        }
    }
    
    tf::TransformListener                                 tf_;
    ros::NodeHandle                                       nh_, root_handle_;

    message_filters::Subscriber<sensor_msgs::PointCloud> sub_;
    tf::MessageFilter<sensor_msgs::PointCloud> tf_filter_;

    filters::FloorFilter *floor_filter_;

    ros::Publisher                                        pointCloudPublisher_;
};

    
int main(int argc, char **argv)
{
  ros::init(argc, argv, "floor_filter_node");

  FloorFilterNode s;
  ros::spin();
    
  return 0;
}
