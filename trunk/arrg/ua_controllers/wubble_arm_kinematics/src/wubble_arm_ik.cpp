//Software License Agreement (BSD License)

//Copyright (c) 2010, Antons Rebguns
//Copyright (c) 2008, Willow Garage, Inc.
//All rights reserved.

//Redistribution and use in source and binary forms, with or without
//modification, are permitted provided that the following conditions
//are met:

// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
// * Neither the name of Willow Garage, Inc. nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.

//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//POSSIBILITY OF SUCH DAMAGE.

#include <angles/angles.h>
#include <wubble_arm_kinematics/wubble_arm_ik.h>

#define IKFAST_NO_MAIN
#include <wubble_arm_kinematics/wubble_arm_ik_gen_f2.cpp>

using namespace angles;
using namespace wubble_arm_kinematics;

WubbleArmIK::WubbleArmIK()
{
}

bool WubbleArmIK::init(const urdf::Model &robot_model, const std::string &root_name, const std::string &tip_name)
{
  int num_joints = 0;
  boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);

  while (link && num_joints < NUM_JOINTS_ARM7DOF)
  {
    boost::shared_ptr<const urdf::Joint> joint = robot_model.getJoint(link->parent_joint->name);

    if (!joint)
    {
      ROS_ERROR("Could not find joint: %s", link->parent_joint->name.c_str());
      return false;
    }

    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      angle_multipliers_.push_back(joint->axis.x * fabs(joint->axis.x) + joint->axis.y * fabs(joint->axis.y) + joint->axis.z * fabs(joint->axis.z));
      ROS_DEBUG("Joint axis: %d, %f, %f, %f", 6 - num_joints, joint->axis.x, joint->axis.y, joint->axis.z);

      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        min_angles_.push_back(joint->safety->soft_lower_limit);
        max_angles_.push_back(joint->safety->soft_upper_limit);
        continuous_joint_.push_back(false);
      }
      else
      {
        min_angles_.push_back(-M_PI);
        max_angles_.push_back(M_PI);
        continuous_joint_.push_back(true);
      }

      addJointToChainInfo(link->parent_joint, solver_info_);
      num_joints++;
    }

    link = robot_model.getLink(link->getParent()->name);
  }

  solver_info_.link_names.push_back(tip_name);

  // We expect order from root to tip, so reverse the order
  std::reverse(angle_multipliers_.begin(), angle_multipliers_.end());
  std::reverse(min_angles_.begin(), min_angles_.end());
  std::reverse(max_angles_.begin(), max_angles_.end());
  std::reverse(solver_info_.limits.begin(), solver_info_.limits.end());
  std::reverse(solver_info_.joint_names.begin(), solver_info_.joint_names.end());
  std::reverse(solver_info_.link_names.begin(), solver_info_.link_names.end());
  std::reverse(continuous_joint_.begin(), continuous_joint_.end());

  if (num_joints != 7)
  {
    ROS_FATAL("WubbleArmIK:: Chain from %s to %s does not have 7 joints", root_name.c_str(), tip_name.c_str());
    return false;
  }

  solution_.resize( NUM_JOINTS_ARM7DOF);

  return true;
}

void WubbleArmIK::addJointToChainInfo(boost::shared_ptr<const urdf::Joint> joint, kinematics_msgs::KinematicSolverInfo &info)
{
  arm_navigation_msgs::JointLimits limit;
  info.joint_names.push_back(joint->name);  //Joints are coming in reverse order
  limit.min_position = joint->safety->soft_lower_limit;
  limit.max_position = joint->safety->soft_upper_limit;

  if (joint->type != urdf::Joint::CONTINUOUS)
  {
    limit.min_position = joint->safety->soft_lower_limit;
    limit.max_position = joint->safety->soft_upper_limit;
    limit.has_position_limits = true;
  }
  else
  {
    limit.min_position = -M_PI;
    limit.max_position = M_PI;
    limit.has_position_limits = false;
  }

  limit.max_velocity = joint->limits->velocity;
  limit.has_velocity_limits = 1;
  info.limits.push_back(limit);
}

void WubbleArmIK::getSolverInfo(kinematics_msgs::KinematicSolverInfo &info)
{
  info = solver_info_;
}

void WubbleArmIK::computeIKUpperArmRoll(const KDL::Frame& g_in, const double& upperarm_roll_initial_guess)
{
  //t1 = upperarm roll is specified
  solution_ik_.clear();

  double t1 = angles::normalize_angle(upperarm_roll_initial_guess);
  if (!checkJointLimits(t1, 2)) { return; }

  IKReal translation[3];
  translation[0] = g_in.p.x();
  translation[1] = g_in.p.y();
  translation[2] = g_in.p.z();

  IKReal rotation[9];
  rotation[0] = g_in.M(0,0); rotation[1] = g_in.M(0,1); rotation[2] = g_in.M(0,2);
  rotation[3] = g_in.M(1,0); rotation[4] = g_in.M(1,1); rotation[5] = g_in.M(1,2);
  rotation[6] = g_in.M(2,0); rotation[7] = g_in.M(2,1); rotation[8] = g_in.M(2,2);

  IKReal free_params[1] = { t1 };

  ROS_DEBUG("Translation (%f,%f,%f)", translation[0], translation[1], translation[2]);
  ROS_DEBUG("Rotation (%f,%f,%f)", rotation[0], rotation[1], rotation[2]);
  ROS_DEBUG("Rotation (%f,%f,%f)", rotation[3], rotation[4], rotation[5]);
  ROS_DEBUG("Rotation (%f,%f,%f)", rotation[6], rotation[7], rotation[8]);

  std::vector<IKSolution> ik_solutions;

  if (ik(translation, rotation, free_params, ik_solutions))
  {
      ROS_DEBUG("Found %zd IK solutions before pruning", ik_solutions.size());
      std::vector<IKReal> sol(NUM_JOINTS_ARM7DOF);

    for (size_t i = 0; i < ik_solutions.size(); ++i)
    {
      std::vector<IKReal> vsolfree(ik_solutions[i].GetFree().size());
      ik_solutions[i].GetSolution(&sol[0], &vsolfree[0]);

      bool within_limits = true;

      // check to see if returned solution are within joint limits
      for (size_t j = 0; j < sol.size(); ++j)
      {
        if (!checkJointLimits(sol[j], j)) { within_limits = false; break; }
        else { solution_[j] = normalize_angle(sol[j]) * angle_multipliers_[j]; }
      }

      if (within_limits)
      {
        solution_ik_.push_back(solution_);
        ROS_DEBUG_STREAM("SOLN " << solution_[0] << " " << solution_[1] << " " <<  solution_[2] << " " << solution_[3] <<  " " << solution_[4] << " " << solution_[5] <<  " " << solution_[6] << std::endl << std::endl);
      }
    }
  }

  ROS_DEBUG("Found %zd IK solutions after pruning", solution_ik_.size());
}

bool WubbleArmIK::checkJointLimits(const std::vector<double> &joint_values)
{
  for (int i = 0; i < NUM_JOINTS_ARM7DOF; ++i)
  {
    if (!checkJointLimits(angles::normalize_angle(joint_values[i] * angle_multipliers_[i]),i))
    {
      return false;
    }
  }

  return true;
}

bool WubbleArmIK::checkJointLimits(const double &joint_value, const int &joint_num)
{
  double jv = angles::normalize_angle(joint_value * angle_multipliers_[joint_num]);

  if (jv < min_angles_[joint_num] || jv > max_angles_[joint_num])
  {
    ROS_DEBUG("Angle %d = %f out of range: (%f,%f)\n", joint_num, joint_value,min_angles_[joint_num], max_angles_[joint_num]);
    return false;
  }

  return true;
}

