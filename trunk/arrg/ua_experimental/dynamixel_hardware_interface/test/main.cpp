
// Author: Antons Rebguns

#include <stdint.h>

#include <sstream>
#include <string>
#include <vector>

#include <dynamixel_hardware_interface/dynamixel_io.h>
#include <dynamixel_hardware_interface/serial_proxy.h>

#include <ros/ros.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "dynamixel_hardware_interface");
    dynamixel_hardware_interface::SerialProxy sp("/dev/ttyUSB0", "ttyUSB0", "1000000");
    sp.connect();
    ros::spin();
    exit(1);
    
    
    dynamixel_hardware_interface::DynamixelIO dxl_io("/dev/ttyUSB0", "1000000");

    int num_motors = 4;
    int motors[] = { 4, 5, 6, 13 };

    for (int i = 0; i < num_motors; ++i)
    {
        int id = motors[i];

        if (dxl_io.ping(id))
        {
            uint16_t model_number, cw_angle, ccw_angle, position;
            uint8_t firmware_version, return_delay_time;
            int16_t velocity;

            dxl_io.getModelNumber(id, model_number);
            dxl_io.getFirmwareVersion(id, firmware_version);
            dxl_io.getReturnDelayTime(id, return_delay_time);
            dxl_io.getCWAngleLimit(id, cw_angle);
            dxl_io.getCCWAngleLimit(id, ccw_angle);
            dxl_io.getPosition(id, position);
            dxl_io.getVelocity(id, velocity);

            ROS_INFO("Pinging motor ID %d successfull, model is %d", id, model_number);
            ROS_INFO("ID %d: fw %d, delay %d, cw %d, ccw %d, pos %d, vel %d", id, firmware_version, return_delay_time, cw_angle, ccw_angle, position, velocity);

            dxl_io.setVelocity(id, 64);
            //dxl_io.setPosition(id, 256);
        }
    }

    int position = 512;
    int velocity = 64;
    
    std::vector<std::vector<int> > value_pairs;

    std::vector<int> pair1;
    pair1.push_back(4);
    pair1.push_back(position);
    pair1.push_back(velocity);

    std::vector<int> pair2;
    pair2.push_back(5);
    pair2.push_back(position);
    pair2.push_back(velocity);

    std::vector<int> pair3;
    pair3.push_back(6);
    pair3.push_back(position);
    pair3.push_back(velocity);

    std::vector<int> pair4;
    pair4.push_back(13);
    pair4.push_back(position);
    pair4.push_back(velocity);

    value_pairs.push_back(pair1);
    value_pairs.push_back(pair2);
    value_pairs.push_back(pair3);
    value_pairs.push_back(pair4);

    dxl_io.setMultiPositionVelocity(value_pairs);

    return 0;
}
