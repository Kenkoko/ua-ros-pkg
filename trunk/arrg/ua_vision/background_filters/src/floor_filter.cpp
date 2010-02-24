#include "background_filters/floor_filter.h"
#include "pluginlib/class_list_macros.h"
#include <sensor_msgs/ChannelFloat32.h>

using namespace filters;

FloorFilter::FloorFilter()
{
}

bool FloorFilter::configure()
{  
    return true;
}

FloorFilter::~FloorFilter()
{
}

bool FloorFilter::update(const sensor_msgs::PointCloud &data_in, sensor_msgs::PointCloud &data_out)
{    
    data_out.header.frame_id = data_in.header.frame_id;
    data_out.header.stamp = data_in.header.stamp;

    for (unsigned int i = 0; i < data_in.channels.size(); i++) {
        sensor_msgs::ChannelFloat32 channel;
        channel.name = data_in.channels[i].name;       
        data_out.channels.push_back(channel);
    }

    for (unsigned int i = 0; i < data_in.points.size(); i++) {
        if (i % 1000 == 0) 
            ROS_DEBUG("%4.2f %4.2f %4.2f", data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);        
    
        if (data_in.points[i].z > 0.05) {
            data_out.points.push_back(data_in.points[i]);
            
            for (unsigned int j = 0; j < data_in.channels.size(); j++) {
                data_out.channels[j].values.push_back(data_in.channels[j].values[i]);
            }
        }     
    }

    return true;
};

PLUGINLIB_REGISTER_CLASS(FloorFilter, filters::FloorFilter, filters::FilterBase<sensor_msgs::PointCloud>)
