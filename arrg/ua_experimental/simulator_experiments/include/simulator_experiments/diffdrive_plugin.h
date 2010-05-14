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
 * Desc: Position2d controller for a Differential drive.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id: Differential_Position2d.hh 8433 2009-11-22 20:15:37Z natepak $
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

/// \addtogroup gazebo_controller
/// \{
/** \defgroup pioneer2dx_position2d pioneer2dx_position2d

  \brief Pioneer 2 DX Position2D controller.

  This is a controller that simulates a Pioneer 2DX motion

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

/// \brief Pioneer 2 DX Position2D controller.
/// This is a controller that simulates a Pioneer 2DX motion
class DiffDrivePlugin : public Controller
{
  /// Constructor
  public: DiffDrivePlugin(Entity *parent );

  /// Destructor
  public: virtual ~DiffDrivePlugin();

  /// Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Save the controller.
  /// \stream Output stream
  protected: void SaveChild(std::string &prefix, std::ostream &stream);

  /// Init the controller
  protected: virtual void InitChild();

  /// \brief Reset the controller
  protected: void ResetChild();

  /// Update the controller
  protected: virtual void UpdateChild();

  /// Finalize the controller
  protected: virtual void FiniChild();

  /// Update the data in the interface
  private: void PutPositionData();

  /// Get the position command from libgazebo
  private: void GetPositionCmd();

  /// The Position interface
  private: PositionIface *myIface;

  /// The parent Model
  private: Model *myParent;

  /// Separation between the wheels
  private: ParamT<float> *wheelSepP;

  /// Diameter of the wheels
  private: ParamT<float> *wheelDiamP;

  ///Torque applied to the wheels
  private: ParamT<float> *torqueP;

  /// Speeds of the wheels
  private: float wheelSpeed[2];

  // Simulation time of the last update
  private: Time prevUpdateTime;

  /// True = enable motors
  private: bool enableMotors;

  private: float odomPose[3];
  private: float odomVel[3];

  private: Joint *joints[2];

  private: PhysicsEngine  *physicsEngine;

  private: ParamT<std::string> *leftJointNameP;
  private: ParamT<std::string> *rightJointNameP;

  // ROS STUFF

  /// \bridf: ros node handle and publisher
  private: ros::NodeHandle* rosnode_;
  private: ros::Publisher pub_;
  private: ros::Subscriber sub_;

  /// \brief A mutex to lock access to fields that are used in message callbacks
  private: boost::mutex lock;

  /// \brief for setting ROS name space
  private: ParamT<std::string> *robotNamespaceP;
  private: std::string robotNamespace;

  /// \brief topic name
  private: ParamT<std::string> *topicNameP;
  private: std::string topicName;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  private: void QueueThread();
  private: boost::thread* callback_queue_thread_;


  // DiffDrive stuff
  private: gazebo::PositionIface *posIface;
  private: void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
  //private: geometry_msgs::Twist::ConstPtr cmd_msg;
  private: float x_;
  private: float rot_;
  private: bool alive_;
};

/** \} */
/// \}

}

#endif

