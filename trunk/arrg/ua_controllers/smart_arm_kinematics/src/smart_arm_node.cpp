/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2010, Arizona Robotics Research Group,
* University of Arizona. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of University of Arizona nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Antons Rebguns
*
*/

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <smart_arm_kinematics/SmartArmIK.h>

#define IKFAST_NO_MAIN
#include "smart_arm_ik.cpp"

bool do_ik(smart_arm_kinematics::SmartArmIK::Request  &req,
           smart_arm_kinematics::SmartArmIK::Response &res)
{
    ROS_DEBUG("smart_arm_ik_service:: received service request");
    tf::TransformListener tf_listener;
    std::vector<IKSolution> vsolutions;
    tf::Stamped<tf::Point> point_stamped;
    
    IKReal eerot[9];
    IKReal eetrans[3];
    std::vector<IKReal> vfree(1);
    eerot[0] = 1.0; eerot[1] = 0.0; eerot[2] = 0.0;
    eerot[3] = 0.0; eerot[4] = 1.0; eerot[5] = 0.0;
    eerot[6] = 0.0; eerot[7] = 0.0; eerot[8] = 1.0;
    vfree[0] = 0.0;
    
    tf::pointStampedMsgToTF(req.goal, point_stamped);
    
    // convert to reference frame of base link of the arm
    if (!tf_listener.canTransform("arm_base_link", point_stamped.frame_id_, point_stamped.stamp_))
    {
        std::string err;
        if (tf_listener.getLatestCommonTime(point_stamped.frame_id_, "arm_base_link", point_stamped.stamp_, &err) != tf::NO_ERROR)
        {
            ROS_ERROR("smart_arm_ik_service:: Cannot transform from '%s' to '%s'. TF said: %s", point_stamped.frame_id_.c_str(), "arm_base_link", err.c_str());
            res.success = false;
            return res.success;
        }
    }
    
    try
    {
        tf_listener.transformPoint("arm_base_link", point_stamped, point_stamped);
    }
    catch(...)
    {
        ROS_ERROR("smart_arm_ik_service:: Cannot transform from '%s' to '%s'", point_stamped.frame_id_.c_str(), "arm_base_link");
        res.success = false;
        return res.success;
    }
    
    eetrans[0] = (float) point_stamped.x();
    eetrans[1] = (float) point_stamped.y();
    eetrans[2] = (float) point_stamped.z();
    
    res.success = ik(eetrans, eerot, &vfree[0], vsolutions);
    
    if( res.success )
    {
        std::vector<IKReal> sol(getNumJoints());
        
        for(size_t i = 0; i < vsolutions.size(); ++i)
        {
            std::vector<IKReal> vsolfree(vsolutions[i].GetFree().size());
            vsolutions[i].GetSolution(&sol[0], vsolfree.size() > 0 ? &vsolfree[0] : NULL);
            
            for( size_t j = 0; j < sol.size(); ++j)
            {
                res.solutions.push_back(sol[j]);
            }
        }
    }
    
    return res.success;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smart_arm_ik_service_node");
    
    std::string joint_names[4];
    
    joint_names[0] = "shoulder_pan_joint";
    joint_names[1] = "shoulder_tilt_joint";
    joint_names[2] = "elbow_tilt_joint";
    joint_names[3] = "wrist_rotate_joint";
    
    ros::NodeHandle n;
    ros::ServiceServer ik_service = n.advertiseService("smart_arm_ik_service", do_ik);
    
    ros::spin();
}

