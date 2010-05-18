/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 * Desc: ROS interface to a Position2d controller for a Differential drive.
 * Author: Daniel Hewlett (adapted from Nathan Koenig)
 * Date: 17 May 2010
 */
#ifndef DIFFDRIVE_PLUGIN_HH
#define DIFFDRIVE_PLUGIN_HH

#include <map>

#include <gazebo/Param.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Model.hh>

// ROS 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{
class Joint;
class Entity;
class PositionIface;

/** \brief Pioneer 2 DX Position2D controller.
 This is a controller that simulates a Pioneer 2DX motion
 TODO: Need to update this documentation block with correct values

 \verbatim
 <controller:pioneer2dx_position2d name="controller-name">
 <leftJoint>left-joint-name</leftJoint>
 <rightJoint>right-join-name</rightJoint>
 <wheelDiameter>diameter_in_meters</wheelDiameter>
 <wheelSeparation>separation_in_meters</wheelSeparation>
 <torque></torque>
 <interface:position name="iface-name"/>
 </controller:pioneer2dx_position2d>
 \endverbatim

 \{
 */

class DiffDrivePlugin : public Controller
{

public:
  DiffDrivePlugin(Entity *parent);
  virtual ~DiffDrivePlugin();

protected:
  virtual void LoadChild(XMLConfigNode *node);
  void SaveChild(std::string &prefix, std::ostream &stream);
  virtual void InitChild();
  void ResetChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:
  void PutPositionData();
  void GetPositionCmd();

  PositionIface *myIface;
  Model *myParent;
  ParamT<float> *wheelSepP;
  ParamT<float> *wheelDiamP;
  ParamT<float> *torqueP;
  float wheelSpeed[2];

  // Simulation time of the last update
  Time prevUpdateTime;

  bool enableMotors;
  float odomPose[3];
  float odomVel[3];

  Joint *joints[2];
  PhysicsEngine *physicsEngine;
  ParamT<std::string> *leftJointNameP;
  ParamT<std::string> *rightJointNameP;

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  boost::mutex lock;

  ParamT<std::string> *robotNamespaceP;
  std::string robotNamespace;

  ParamT<std::string> *topicNameP;
  std::string topicName;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  void QueueThread();
  boost::thread* callback_queue_thread_;

  // DiffDrive stuff
  gazebo::PositionIface *posIface;
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  float x_;
  float rot_;
  bool alive_;
};

}

#endif

