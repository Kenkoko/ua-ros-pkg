/*
    Copyright (c) 2011, Antons Rebguns <email>
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef DYNAMIXEL_HARDWARE_INTERFACE_SINGLE_JOINT_CONTROLLER_H
#define DYNAMIXEL_HARDWARE_INTERFACE_SINGLE_JOINT_CONTROLLER_H

#include <map>
#include <string>
#include <cmath>

#include <dynamixel_hardware_interface/dynamixel_io.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>
#include <dynamixel_msgs/MotorStateList.h>

namespace controller
{

class SingleJointController
{
public:
    SingleJointController() {};
    virtual ~SingleJointController() {};
    
    virtual bool initialize(std::string name,
                            std::string port_namespace,
                            dynamixel_hardware_interface::DynamixelIO* dxl_io)
    {
        name_ = name;
        
        try
        {
            c_nh_ = ros::NodeHandle(nh_, name_);
        }
        catch(std::exception &e)
        {
            ROS_ERROR("Exception thrown while constructing nodehandle for controller with name '%s':\n%s", name_.c_str(), e.what());
            return false;
        }
        catch(...)
        {
            ROS_ERROR("Exception thrown while constructing nodehandle for controller with name '%s'", name_.c_str());
            return false;
        }

        port_namespace_ = port_namespace;
        dxl_io_ = dxl_io;

        if (!c_nh_.getParam("joint", joint_))
        {
            ROS_ERROR("%s: joint name is not specified", name_.c_str());
            return false;
        }
        
        c_nh_.param("max_velocity", max_velocity_, 3.0);
        c_nh_.param("init_velocity", init_velocity_, 1.0);
        
        return true;
    }
    
    const dynamixel_msgs::JointState& getJointState() { return joint_state_; }
    dynamixel_hardware_interface::DynamixelIO* getPort() { return dxl_io_; }
    
    std::string getName() { return name_; }
    std::string getJointName() { return joint_; }
    std::string getPortNamespace() { return port_namespace_; }
    
    virtual void start()
    {
        motor_states_sub_ = nh_.subscribe("motor_states/" + port_namespace_, 50, &SingleJointController::processMotorStates, this);
        joint_command_sub_ = c_nh_.subscribe("command", 50, &SingleJointController::processCommand, this);
        joint_state_pub_ = c_nh_.advertise<dynamixel_msgs::JointState>("state", 50);
    }
    
    virtual void stop()
    {
        motor_states_sub_.shutdown();
        joint_command_sub_.shutdown();
        joint_state_pub_.shutdown();
    }
    
    virtual void setVelocity(double velocity) = 0;
    
    virtual void processMotorStates(const dynamixel_msgs::MotorStateListConstPtr& msg) = 0;
    virtual void processCommand(const std_msgs::Float64ConstPtr& msg) = 0;
    
    virtual std::vector<int> getMotorIDs() = 0;
    virtual std::vector<std::vector<int> > getWritableVals(double position, double velocity) = 0;
    
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle c_nh_;
    
    std::string name_;
    std::string port_namespace_;
    dynamixel_hardware_interface::DynamixelIO* dxl_io_;
    
    std::string joint_;
    dynamixel_msgs::JointState joint_state_;
    
    double init_velocity_;
    double max_velocity_;
    double min_velocity_;
    
    double min_angle_radians_;
    double max_angle_radians_;
    
    int init_position_encoder_;
    int min_angle_encoder_;
    int max_angle_encoder_;
    bool flipped_;
    
    double encoder_ticks_per_radian_;
    double radians_per_encoder_tick_;
    double velocity_per_encoder_tick_;
    double motor_max_velocity_;
        
    ros::Subscriber motor_states_sub_;
    ros::Subscriber joint_command_sub_;
    ros::Publisher joint_state_pub_;

    uint16_t convertToEncoder(double angle_in_radians)
    {
        double angle_in_encoder = angle_in_radians * encoder_ticks_per_radian_;
        angle_in_encoder = flipped_ ? init_position_encoder_ - angle_in_encoder : init_position_encoder_ + angle_in_encoder;
        return (int) (round(angle_in_encoder));
    }

    double convertToRadians(int angle_in_encoder)
    {
        double angle_in_radians = flipped_ ? init_position_encoder_ - angle_in_encoder : angle_in_encoder - init_position_encoder_;
        return angle_in_radians * radians_per_encoder_tick_;
    }

private:
    SingleJointController(const SingleJointController &c);
    SingleJointController& operator =(const SingleJointController &c);
};

}

#endif  // DYNAMIXEL_HARDWARE_INTERFACE_SINGLE_JOINT_CONTROLLER_H