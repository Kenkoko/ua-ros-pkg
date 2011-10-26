/*
 *    Copyright (c) 2011, Antons Rebguns <email>
 *    All rights reserved.
 * 
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 * 
 *    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sstream>
#include <string>
#include <vector>
#include <numeric>
#include <XmlRpcValue.h>

#include <wubble2_gripper_controller/wubble_gripper_action_controller.h>
#include <wubble2_gripper_controller/WubbleGripperAction.h>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <dynamixel_hardware_interface/dynamixel_const.h>
#include <dynamixel_hardware_interface/dynamixel_io.h>

#include <dynamixel_hardware_interface/single_joint_controller.h>
#include <dynamixel_hardware_interface/multi_joint_controller.h>
#include <dynamixel_hardware_interface/joint_trajectory_action_controller.h>
#include <dynamixel_hardware_interface/JointState.h>


PLUGINLIB_DECLARE_CLASS(wubble2_gripper_controller,
                        WubbleGripperActionController,
                        controller::WubbleGripperActionController,
                        controller::MultiJointController)

namespace controller
{
    
WubbleGripperActionController::WubbleGripperActionController()
{
    terminate_gripper_opening_ = false;
    terminate_gripper_monitor_ = false;
}

WubbleGripperActionController::~WubbleGripperActionController()
{
}

bool WubbleGripperActionController::initialize(std::string name, std::vector<SingleJointController*> deps)
{
    if (!MultiJointController::initialize(name, deps)) { return false; }

    std::string l_controller_name;
    std::string r_controller_name;
    
    c_nh_.param<std::string>("left_controller_name", l_controller_name, "left_finger_controller");
    c_nh_.param<std::string>("right_controller_name", r_controller_name, "right_finger_controller");

    std::string l_joint_name;
    std::string r_joint_name;
    
    for (size_t i = 0; i < deps.size(); ++i)
    {
        std::string cn = deps[i]->getName();
        std::string jn = deps[i]->getJointName();
        if (cn == l_controller_name) { l_joint_name = jn; }
        else if (cn == r_controller_name) { r_joint_name = jn; }
    }
    
    if (l_joint_name.empty() || r_joint_name.empty())
    {
        ROS_ERROR("%s: bad gripper configuration, unable to find controllers", name_.c_str());
        return false;
    }
    
    l_finger_controller_ = joint_to_controller_[l_joint_name];
    r_finger_controller_ = joint_to_controller_[r_joint_name];
    
    l_finger_state_ = l_finger_controller_->getJointState();
    r_finger_state_ = r_finger_controller_->getJointState();
    
    // left and right finger are assumed to be connected to the same port
    dxl_io_ = l_finger_controller_->getPort();
    
    l_max_velocity_ = l_finger_controller_->getMaxVelocity();
    r_max_velocity_ = r_finger_controller_->getMaxVelocity();

    return true;
}

void WubbleGripperActionController::start()
{
    // Publishers
    gripper_opening_pub_ = nh_.advertise<std_msgs::Float64>("gripper_opening", 100);
    l_finger_ground_distance_pub_ = nh_.advertise<std_msgs::Float64>("left_finger_ground_distance", 100);
    r_finger_ground_distance_pub_ = nh_.advertise<std_msgs::Float64>("right_finger_ground_distance", 100);
    
    // Pressure sensors
    // 0-3 - left finger
    // 4-7 - right finger
    int num_sensors = 8;
    pressure_.resize(num_sensors);
    pressure_subs_.resize(num_sensors);
    
    for (int i = 0; i < num_sensors; ++i)
    {
        pressure_subs_[i] = nh_.subscribe<phidgets_ros::Float64Stamped>("/interface_kit/124427/sensor/" + boost::lexical_cast<std::string>(i), 100,
                                                                        boost::bind(&WubbleGripperActionController::processPressureSensors, this, _1, i));
    }
    
    // pressure sensors are at these values when no external pressure is applied
    l_zero_pressure_.push_back(0.0);
    l_zero_pressure_.push_back(0.0);
    l_zero_pressure_.push_back(0.0);
    l_zero_pressure_.push_back(0.0);
    
    r_zero_pressure_.push_back(0.0);
    r_zero_pressure_.push_back(0.0);
    r_zero_pressure_.push_back(130.0);
    r_zero_pressure_.push_back(0.0);
    
    lr_zero_pressure_.insert(lr_zero_pressure_.end(), l_zero_pressure_.begin(), l_zero_pressure_.end());
    lr_zero_pressure_.insert(lr_zero_pressure_.end(), r_zero_pressure_.begin(), r_zero_pressure_.end());
    
    l_total_pressure_pub_ = nh_.advertise<std_msgs::Float64>("left_finger_pressure", 100);
    r_total_pressure_pub_ = nh_.advertise<std_msgs::Float64>("right_finger_pressure", 100);
    lr_total_pressure_pub_ = nh_.advertise<std_msgs::Float64>("total_pressure", 100);
    
    close_gripper_ = false;
    dynamic_torque_control_ = false;
    lower_pressure_ = 800.0;
    upper_pressure_ = 1000.0;
    
    // IR sensor
    gripper_ir_pub_ = nh_.advertise<std_msgs::Float64>("gripper_distance_sensor", 100);
    ir_distance_ = 0.0;
    ir_sensor_sub_ = nh_.subscribe("/interface_kit/106950/sensor/7", 100, &WubbleGripperActionController::processIRSensor, this);
  
    // Temperature monitor and torque control thread
    gripper_monitor_thread_ = new boost::thread(boost::bind(&WubbleGripperActionController::gripperMonitor, this));
    
    // Start gripper opening monitoring thread
    gripper_opening_thread_ = new boost::thread(boost::bind(&WubbleGripperActionController::calculateGripperOpening, this));

    action_server_.reset(new WGAS(nh_, "wubble_gripper_action",
                                  boost::bind(&WubbleGripperActionController::processGripperAction, this, _1),
                                  false));
    action_server_->start();

    ROS_INFO("%s: ready to accept goals", name_.c_str());
}

void WubbleGripperActionController::stop()
{
    if (gripper_opening_thread_)
    {
        {
            boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
            terminate_gripper_opening_ = true;
        }
        gripper_opening_thread_->join();
        delete gripper_opening_thread_;
    }
    
    if (gripper_monitor_thread_)
    {
        {
            boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
            terminate_gripper_monitor_ = true;
        }
        gripper_monitor_thread_->join();
        delete gripper_monitor_thread_;
    }
    
    gripper_opening_pub_.shutdown();
    l_finger_ground_distance_pub_.shutdown();
    r_finger_ground_distance_pub_.shutdown();
    
    for (size_t i = 0; i < pressure_subs_.size(); ++i)
    {
        pressure_subs_[i].shutdown();
    }
    
    l_total_pressure_pub_.shutdown();
    r_total_pressure_pub_.shutdown();
    lr_total_pressure_pub_.shutdown();
    gripper_ir_pub_.shutdown();
    ir_sensor_sub_.shutdown();
    
    action_server_->shutdown();
}

void WubbleGripperActionController::processPressureSensors(const phidgets_ros::Float64StampedConstPtr& msg, int id)
{
    pressure_[id] = msg->data;
}

void WubbleGripperActionController::processIRSensor(const phidgets_ros::Float64StampedConstPtr& msg)
{
    // Given a raw sensor value from Sharp IR sensor (4-30cm model)
    // returns an actual distance in meters.
    if (msg->data < 80.0) { ir_distance_ = 1.0; }               // too far
    else if (msg->data > 530.0) { ir_distance_ = 0.0; }         // too close
    else { ir_distance_ =  20.76 / (msg->data - 11.0); }        // just right

    std_msgs::Float64 ir;
    ir.data = ir_distance_;
    gripper_ir_pub_.publish(ir);
}

void WubbleGripperActionController::processGripperAction(const wubble2_gripper_controller::WubbleGripperGoalConstPtr& goal)
{
    wubble2_gripper_controller::WubbleGripperResult result;
    ros::Rate r(500);
    ros::Duration timeout(2.0);
    
    if (goal->command == wubble2_gripper_controller::WubbleGripperGoal::CLOSE_GRIPPER)
    {
        dynamic_torque_control_ = goal->dynamic_torque_control;
        lower_pressure_ = goal->pressure_lower;
        upper_pressure_ = goal->pressure_upper;
        
        double desired_torque = activateGripper(goal->command, goal->torque_limit);
        
        if (!dynamic_torque_control_)
        {
            ros::Time start_time = ros::Time::now();
            
            while (ros::ok() &&
                   ros::Time::now() - start_time < timeout &&
                   (!within_tolerance(l_finger_state_.load, -goal->torque_limit, 0.01) ||
                    !within_tolerance(r_finger_state_.load, -goal->torque_limit, 0.01)))
            {
                r.sleep();
            }
        }
        else
        {
            if (desired_torque > 1e-3) { ros::Duration(1.0 / desired_torque).sleep(); }
        }
            
        close_gripper_ = true;
        action_server_->setSucceeded();
    }
    else if (goal->command == wubble2_gripper_controller::WubbleGripperGoal::OPEN_GRIPPER)
    {
        close_gripper_ = false;
        activateGripper(goal->command, goal->torque_limit);
        
        ros::Time start_time = ros::Time::now();
        while (ros::ok() &&
               ros::Time::now() - start_time < timeout &&
               (l_finger_state_.position < 0.8 ||
                r_finger_state_.position > -0.8))
        {
            r.sleep();
        }
            
        sendMotorCommand(0.5, -0.5);
        action_server_->setSucceeded();
    }
    else
    {
        std::string msg = "Unrecognized command: " + boost::lexical_cast<std::string>(goal->command);
        ROS_ERROR("%s: %s", name_.c_str(), msg.c_str());
        action_server_->setAborted(result, msg);
    }
}

double WubbleGripperActionController::activateGripper(int command, double torque_limit)
{
    double l_desired_torque = 0.0;
    double r_desired_torque = 0.0;
    
    if (command == wubble2_gripper_controller::WubbleGripperGoal::CLOSE_GRIPPER)
    {
        l_desired_torque = -torque_limit * l_max_velocity_;
        r_desired_torque =  torque_limit * r_max_velocity_;
    }
    else // assume opening
    {
        l_desired_torque =  torque_limit * l_max_velocity_;
        r_desired_torque = -torque_limit * r_max_velocity_;
    }
        
    sendMotorCommand(l_desired_torque, r_desired_torque);
    
    return std::max<double>(l_desired_torque, r_desired_torque);
}

void WubbleGripperActionController::calculateGripperOpening()
{
    ros::Duration timeout(5);
    ros::Time last_reported(0);
    std_msgs::Float64 msg;
    ros::Rate r(50);
    
    while (ros::ok())
    {
        {
            boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
            if (terminate_gripper_opening_) { break; }
        }

        try
        {
            std::string map_frame_id("base_footprint");
            std::string palm_frame_id("L7_wrist_roll_link");
            std::string l_fingertip_frame_id("left_fingertip_link");
            std::string r_fingertip_frame_id("right_fingertip_link");
            
            tf_listener_.waitForTransform(palm_frame_id, l_fingertip_frame_id, ros::Time(0), ros::Duration(0.2));
            tf_listener_.waitForTransform(palm_frame_id, r_fingertip_frame_id, ros::Time(0), ros::Duration(0.2));
            
            tf::StampedTransform lt;
            tf::StampedTransform rt;
            tf_listener_.lookupTransform(palm_frame_id, l_fingertip_frame_id, ros::Time(0), lt);
            tf_listener_.lookupTransform(palm_frame_id, r_fingertip_frame_id, ros::Time(0), rt);
            
            btVector3 l_pos = lt.getOrigin();
            btVector3 r_pos = rt.getOrigin();
            
            double dx = l_pos[0] - r_pos[0];
            double dy = l_pos[1] - r_pos[1];
            double dz = l_pos[2] - r_pos[2];
            double dist = sqrt(dx*dx + dy*dy + dz*dz);
            
            msg.data = dist;
            gripper_opening_pub_.publish(msg);
            
            tf_listener_.lookupTransform(map_frame_id, l_fingertip_frame_id, ros::Time(0), lt);
            tf_listener_.lookupTransform(map_frame_id, r_fingertip_frame_id, ros::Time(0), rt);
            
            l_pos = lt.getOrigin();
            r_pos = rt.getOrigin();
            
            msg.data = l_pos[2];
            l_finger_ground_distance_pub_.publish(msg);
            
            msg.data = r_pos[2];
            r_finger_ground_distance_pub_.publish(msg);
        }
        catch (const std::runtime_error &e)
        {
            ROS_ERROR_THROTTLE(1, "%s: %s", name_.c_str(), e.what());
        }
                
        r.sleep();
    }
}

void WubbleGripperActionController::gripperMonitor()
{
    ROS_INFO("%s: Gripper temperature monitor and torque control thread started successfully", name_.c_str());
    bool motors_overheating = false;
    double max_pressure = 8000.0;
    std_msgs::Float64 msg;
    ros::Rate r(150);
    
    while (ros::ok())
    {
        {
            boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
            if (terminate_gripper_monitor_) { break; }
        }
        
        double l_total_pressure = std::max<double>(0.0,
                                                   std::accumulate(pressure_.begin(), pressure_.begin()+4, 0) -
                                                   std::accumulate(l_zero_pressure_.begin(), l_zero_pressure_.end(), 0));
        double r_total_pressure = std::max<double>(0.0,
                                                   std::accumulate(pressure_.begin()+4, pressure_.end(), 0) -
                                                   std::accumulate(r_zero_pressure_.begin(), r_zero_pressure_.end(), 0));
        double pressure = l_total_pressure + r_total_pressure;
        
        msg.data = l_total_pressure;
        l_total_pressure_pub_.publish(msg);
        
        msg.data = r_total_pressure;
        r_total_pressure_pub_.publish(msg);
        
        msg.data = pressure;
        lr_total_pressure_pub_.publish(msg);
        
        //----------------------- TEMPERATURE MONITOR ---------------------------//
        std::vector<int> lt = l_finger_state_.motor_temps;
        int l_temp = *std::max_element(lt.begin(), lt.end());
        
        std::vector<int> rt = r_finger_state_.motor_temps;
        int r_temp = *std::max_element(rt.begin(), rt.end());
        
        if (l_temp >= 75 || r_temp >= 75)
        {
            if (!motors_overheating)
            {
                ROS_WARN("Disabling gripper motors torque [LM: %dC, RM: %dC]", l_temp, r_temp);
                sendMotorCommand(-0.5, 0.5);
                motors_overheating = true;
            }
        }
        else
        {
            motors_overheating = false;
        }
            
        /*#########################################################################*/
        
        // don't do torque control if not requested or
        // when the gripper is open
        // or when motors are too hot
        if (!dynamic_torque_control_ || !close_gripper_ || motors_overheating)
        {
            r.sleep();
            continue;
        }
            
        //----------------------- TORQUE CONTROL -------------------------------//
        double l_current = l_finger_state_.target_position;
        double r_current = r_finger_state_.target_position;
        
        if (pressure > upper_pressure_)   // release
        {
            double pressure_change_step = fabs(pressure - upper_pressure_) / max_pressure;
            double l_goal = std::min( l_max_velocity_, l_current + pressure_change_step);
            double r_goal = std::max(-r_max_velocity_, r_current - pressure_change_step);
            
            if (l_goal < l_max_velocity_ || r_goal > -r_max_velocity_)
            {
                if (close_gripper_)
                {
                    sendMotorCommand(l_goal, r_goal);
                    ROS_DEBUG(">MAX pressure is %.2f, LT: %.2f, RT: %.2f, step is %.2f", pressure, l_current, r_current, pressure_change_step);
                }
            }
        }
        else if (pressure < lower_pressure_) // squeeze
        {
            double pressure_change_step = fabs(pressure - lower_pressure_) / max_pressure;
            double l_goal = std::max(-l_max_velocity_, l_current - pressure_change_step);
            double r_goal = std::min( r_max_velocity_, r_current + pressure_change_step);
            
            if (l_goal > -l_max_velocity_ || r_goal < r_max_velocity_)
            {
                if (close_gripper_)
                {
                    sendMotorCommand(l_goal, r_goal);
                    ROS_DEBUG("<MIN pressure is %.2f, LT: %.2f, RT: %.2f, step is %.2f", pressure, l_current, r_current, pressure_change_step);
                }
            }
        }
        /*########################################################################*/

        r.sleep();
    }
}

void WubbleGripperActionController::sendMotorCommand(double l_desired_torque, double r_desired_torque)
{
    std::vector<std::vector<int> > l = l_finger_controller_->getRawMotorCommands(0.0, l_desired_torque);
    std::vector<std::vector<int> > r = r_finger_controller_->getRawMotorCommands(0.0, r_desired_torque);

    l.push_back(r[0]);
    
    dxl_io_->setMultiVelocity(l);
}

}