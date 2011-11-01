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

#ifndef WUBBLE_GRIPPER_ACTION_CONTROLLER_H
#define WUBBLE_GRIPPER_ACTION_CONTROLLER_H

#include <map>
#include <string>

#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <wubble2_gripper_controller/WubbleGripperAction.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_listener.h>

#include <dynamixel_hardware_interface/single_joint_controller.h>
#include <dynamixel_hardware_interface/multi_joint_controller.h>

#include <phidgets_ros/Float64Stamped.h>

namespace controller
{

class WubbleGripperActionController : public MultiJointController
{
public:
    WubbleGripperActionController();
    virtual ~WubbleGripperActionController();
    
    bool initialize(std::string name, std::vector<SingleJointController*> deps);
    
    void start();
    void stop();
    
    void processPressureSensors(const phidgets_ros::Float64StampedConstPtr& msg, int id);
    void processIRSensor(const phidgets_ros::Float64StampedConstPtr& msg);
    void processGripperAction(const wubble2_gripper_controller::WubbleGripperGoalConstPtr& goal);
    double activateGripper(int command, double torque_limit);
    void calculateGripperOpening();
    void gripperMonitor();

private:
    typedef actionlib::SimpleActionServer<wubble2_gripper_controller::WubbleGripperAction> WGAS;
    boost::scoped_ptr<WGAS> action_server_;
    
    boost::mutex terminate_mutex_;
    
    boost::thread* gripper_opening_thread_;
    bool terminate_gripper_opening_;
    
    boost::thread* gripper_monitor_thread_;
    bool terminate_gripper_monitor_;

    tf::TransformListener tf_listener_;
    
    ros::Publisher gripper_opening_pub_;
    ros::Publisher l_finger_ground_distance_pub_;
    ros::Publisher r_finger_ground_distance_pub_;
    ros::V_Subscriber pressure_subs_;
    
    ros::Publisher l_total_pressure_pub_;
    ros::Publisher r_total_pressure_pub_;
    ros::Publisher lr_total_pressure_pub_;

    std::vector<double> pressure_;
    std::vector<double> l_zero_pressure_;
    std::vector<double> r_zero_pressure_;
    std::vector<double> lr_zero_pressure_;

    // pressure sensors: 0-3 - left finger, 4-7 - right finger
    const int num_fsrs_;
    
    // maximum value fsr can report
    const double max_fsr_value_;
    
    // absolute maximum combined pressure reported by all sensors in the gripper
    const double max_total_pressure_;
    
    bool close_gripper_;
    bool dynamic_torque_control_;
    double lower_pressure_;
    double upper_pressure_;
    double ir_distance_;
    
    ros::Publisher gripper_ir_pub_;
    ros::Subscriber ir_sensor_sub_;
    
    const dynamixel_hardware_interface::JointState* l_finger_state_;
    const dynamixel_hardware_interface::JointState* r_finger_state_;
    
    SingleJointController* l_finger_controller_;
    SingleJointController* r_finger_controller_;
    
    dynamixel_hardware_interface::DynamixelIO* dxl_io_;
    
    double l_max_velocity_;
    double r_max_velocity_;

    void sendMotorCommand(double l_desired_torque, double r_desired_torque);
    
    inline bool within_tolerance(double a, double b, double tolerance)
    {
        return fabs(a - b) < tolerance;
    }

};

}

#endif  // WUBBLE_GRIPPER_ACTION_CONTROLLER_H
