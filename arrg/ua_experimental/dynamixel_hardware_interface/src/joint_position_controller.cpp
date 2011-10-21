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

#include <sstream>
#include <XmlRpcValue.h>

#include <dynamixel_hardware_interface/dynamixel_const.h>
#include <dynamixel_hardware_interface/dynamixel_io.h>
#include <dynamixel_hardware_interface/single_joint_controller.h>
#include <dynamixel_hardware_interface/joint_position_controller.h>
#include <dynamixel_hardware_interface/MotorState.h>
#include <dynamixel_hardware_interface/SetVelocity.h>
#include <dynamixel_hardware_interface/TorqueEnable.h>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Float64.h>

PLUGINLIB_DECLARE_CLASS(dynamixel_hardware_interface,
                        JointPositionController,
                        controller::JointPositionController,
                        controller::SingleJointController)

namespace controller
{

bool JointPositionController::initialize(std::string name,
                                         std::string port_namespace,
                                         dynamixel_hardware_interface::DynamixelIO* dxl_io)
{
    if (!SingleJointController::initialize(name, port_namespace, dxl_io)) { return false; }

    c_nh_.getParam("motor/id", motor_id_);

    joint_state_.name = joint_;
    joint_state_.motor_ids.push_back(motor_id_);
    joint_state_.motor_temps.resize(1);

    std::stringstream ss;
    ss << "dynamixel/" << port_namespace_ << "/";
    
    std::string prefix = ss.str();
    XmlRpc::XmlRpcValue available_ids;
    
    nh_.getParam(prefix + "connected_ids", available_ids);
    
    if (available_ids.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        ROS_ERROR("%s: [%s] paramater is not an array", (prefix + "connected_ids").c_str(), name_.c_str());
        return false;
    }
    
    bool found_motor_id = false;
    
    for (int i = 0; i < available_ids.size(); ++i)
    {
        XmlRpc::XmlRpcValue val = available_ids[i];
        if (motor_id_ == (int) val) { found_motor_id = true; break; }
    }
    
    if (!found_motor_id)
    {
        ROS_ERROR("%s: motor id %d is not connected to port %s or it is connected and not responding",
                  name_.c_str(), motor_id_, port_namespace_.c_str());
        return false;
    }
    
    ss  << motor_id_ << "/";
    prefix = ss.str();
    
    /***************** Joint velocity related parameters **********************/
    nh_.getParam(prefix + "max_velocity", motor_max_velocity_);
    nh_.getParam(prefix + "radians_second_per_encoder_tick", velocity_per_encoder_tick_);
    min_velocity_ = velocity_per_encoder_tick_;
    
    if (max_velocity_ > motor_max_velocity_)
    {
        ROS_WARN("%s: requested maximum joint velocity exceeds motor capabilities (%f > %f)", name_.c_str(), max_velocity_, motor_max_velocity_);
        max_velocity_ = motor_max_velocity_;
    }
    else if (max_velocity_ < min_velocity_)
    {
        ROS_WARN("%s: requested maximum joint velocity is too small (%f < %f)", name_.c_str(), max_velocity_, min_velocity_);
        max_velocity_ = min_velocity_;
    }

    motor_cache_ = dxl_io_->getCachedParameters(motor_id_);
    if (motor_cache_ == NULL) { return false; }

    int encoder_resolution;
    nh_.getParam(prefix + "encoder_resolution", encoder_resolution);
    
    if (motor_cache_->cw_angle_limit == 0 && motor_cache_->ccw_angle_limit == 0)
    {
        ROS_WARN("%s: motor %d is not set to position control mode, setting motor to position control mode", name_.c_str(), motor_id_);
        
        if (!dxl_io_->setAngleLimits(motor_id_, 0, encoder_resolution-1))
        {
            ROS_ERROR("%s: unable to set motor to position control mode", name_.c_str());
            return false;
        }
    }

    /***************** Motor related parameters **********************/
    c_nh_.getParam("motor/init", init_position_encoder_);
    c_nh_.getParam("motor/min", min_angle_encoder_);
    c_nh_.getParam("motor/max", max_angle_encoder_);
    
    flipped_ = min_angle_encoder_ > max_angle_encoder_;
    
    nh_.getParam(prefix + "radians_per_encoder_tick", radians_per_encoder_tick_);
    nh_.getParam(prefix + "encoder_ticks_per_radian", encoder_ticks_per_radian_);
    
    if (flipped_)
    {
        min_angle_radians_ = (init_position_encoder_ - min_angle_encoder_) * radians_per_encoder_tick_;
        max_angle_radians_ = (init_position_encoder_ - max_angle_encoder_) * radians_per_encoder_tick_;
    }
    else
    {
        min_angle_radians_ = (min_angle_encoder_ - init_position_encoder_) * radians_per_encoder_tick_;
        max_angle_radians_ = (max_angle_encoder_ - init_position_encoder_) * radians_per_encoder_tick_;
    }

    setVelocity(INIT_VELOCITY);
    return true;
}

void JointPositionController::start()
{
    SingleJointController::start();
}

void JointPositionController::stop()
{
    SingleJointController::stop();
}

std::vector<int> JointPositionController::getMotorIDs()
{
    return std::vector<int>(1, motor_id_);
}

std::vector<std::vector<int> > JointPositionController::getRawMotorCommands(double position, double velocity)
{
    std::vector<std::vector<int> > value_pairs;

    std::vector<int> pair;
    pair.push_back(motor_id_);
    pair.push_back(posRad2Enc(position));
    pair.push_back(velRad2Enc(velocity));
    
    value_pairs.push_back(pair);

    return value_pairs;
}

void JointPositionController::setVelocity(double velocity)
{
    std::vector<int> pair;
    pair.push_back(motor_id_);
    pair.push_back(velRad2Enc(velocity));
    
    std::vector<std::vector<int> > mcv;
    mcv.push_back(pair);

    dxl_io_->setMultiVelocity(mcv);
}

void JointPositionController::processMotorStates(const dynamixel_hardware_interface::MotorStateListConstPtr& msg)
{
    dynamixel_hardware_interface::MotorState state;
    
    for (size_t i = 0; i < msg->motor_states.size(); ++i)
    {
        if (motor_id_ == msg->motor_states[i].id)
        {
            state = msg->motor_states[i];
            break;
        }
    }
    
    if (state.id != motor_id_)
    {
        ROS_ERROR("%s: motor id %d not found", name_.c_str(), motor_id_);
        return;
    }
    
    joint_state_.header.stamp = ros::Time(state.timestamp);
    joint_state_.motor_temps[0] = state.temperature;
    joint_state_.target_position = convertToRadians(state.target_position);
    joint_state_.target_velocity = ((double)state.target_velocity / dynamixel_hardware_interface::DXL_MAX_VELOCITY_ENCODER) * motor_max_velocity_;
    joint_state_.position = convertToRadians(state.position);
    joint_state_.velocity = ((double)state.velocity / dynamixel_hardware_interface::DXL_MAX_VELOCITY_ENCODER) * motor_max_velocity_;
    joint_state_.load = (double)state.load / dynamixel_hardware_interface::DXL_MAX_LOAD_ENCODER;
    joint_state_.moving = state.moving;
    
    joint_state_pub_.publish(joint_state_);
}

void JointPositionController::processCommand(const std_msgs::Float64ConstPtr& msg)
{
    std::vector<int> pair;
    pair.push_back(motor_id_);
    pair.push_back(posRad2Enc(msg->data));
    
    std::vector<std::vector<int> > mcv;
    mcv.push_back(pair);
    
    dxl_io_->setMultiPosition(mcv);
}

bool JointPositionController::processSetVelocity(dynamixel_hardware_interface::SetVelocity::Request& req,
                                                 dynamixel_hardware_interface::SetVelocity::Request& res)
{
    setVelocity(req.velocity);
    return true;
}

bool JointPositionController::processTorqueEnable(dynamixel_hardware_interface::TorqueEnable::Request& req,
                                                  dynamixel_hardware_interface::TorqueEnable::Request& res)
{
    std::vector<int> pair;
    pair.push_back(motor_id_);
    pair.push_back(req.torque_enable);
    
    std::vector<std::vector<int> > mcv;
    mcv.push_back(pair);
    
    dxl_io_->setMultiTorqueEnabled(mcv);
    return true;
}

uint16_t JointPositionController::posRad2Enc(double pos_rad)
{
    if (pos_rad < min_angle_radians_) { pos_rad = min_angle_radians_; }
    if (pos_rad > max_angle_radians_) { pos_rad = max_angle_radians_; }
    return convertToEncoder(pos_rad);
}

uint16_t JointPositionController::velRad2Enc(double vel_rad)
{
    if (vel_rad < min_velocity_) { vel_rad = min_velocity_; }
    if (vel_rad > max_velocity_) { vel_rad = max_velocity_; }
    
    return std::max<uint16_t>(1, (uint16_t) round(vel_rad / velocity_per_encoder_tick_));
}

}
