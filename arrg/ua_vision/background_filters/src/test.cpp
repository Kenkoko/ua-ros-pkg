#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <boost/thread/thread.hpp>
#include <math.h>
#include <background_filters/Test.h>

#include <sys/types.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>

using namespace std;

int main (int argc, char **argv)
{
    ros::init(argc, argv, "test_send_cvmat");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<background_filters::Test>("test_mat", 1);
    
    float a[] = { 1.0, 2.0, 3.0,
                  4.0, 5.0, 6.0,
                  7.0, 8.0, 9.0 };
    CvMat m;
    cvInitMatHeader(&m, 3, 3, CV_32FC1, a);
    
    vector<float> v;
    
    for (int row = 0; row < m.rows; ++row)
    {
        const float* ptr = (const float*)(m.data.ptr + row * m.step);
        
        for (int col = 0; col < m.cols; ++col)
        {
            v.push_back(*ptr++);
        }
    }
    
    background_filters::Test t;
    t.mat = v;
    
    ros::Rate(1.0);
    while (ros::ok())
    {
        pub.publish(v);
    }
    
    ros::spin();
    
    return 0;
}

