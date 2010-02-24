#ifndef FLOOR_FILTER_H
#define FLOOR_FILTER_H
#include <stdint.h>
#include <cstring>
#include <stdio.h>
#include <boost/scoped_ptr.hpp>
#include "filters/filter_base.h"
#include "sensor_msgs/PointCloud.h"

namespace filters {

class FloorFilter : public FilterBase<sensor_msgs::PointCloud>
{
public:
    FloorFilter();
    ~FloorFilter();
    virtual bool configure();
    virtual bool update( const sensor_msgs::PointCloud &data_in, sensor_msgs::PointCloud &data_out);
};

}

#endif
