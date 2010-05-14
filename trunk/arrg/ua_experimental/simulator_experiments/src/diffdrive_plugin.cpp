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
 * SVN info: $Id: Differential_Position2d.cc 8567 2010-03-09 18:20:08Z natepak $
 */

//#include "XMLConfig.hh"
//#include "Model.hh"
//#include "Global.hh"
//#include "Joint.hh"
//#include "World.hh"
//#include "Simulator.hh"
//#include "PhysicsEngine.hh"

#include <algorithm>
#include <assert.h>

#include <simulator_experiments/diffdrive_plugin.h>

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/Quatern.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/PhysicsEngine.hh>

#include <boost/bind.hpp>


using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("diffdrive_plugin", DiffDrivePlugin);

enum {RIGHT, LEFT};

////////////////////////////////////////////////////////////////////////////////
// Constructor
DiffDrivePlugin::DiffDrivePlugin(Entity *parent )
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Differential_Position2d controller requires a Model as its parent");

  this->enableMotors = true;

  this->wheelSpeed[RIGHT] = 0;
  this->wheelSpeed[LEFT] = 0;

  this->prevUpdateTime = Simulator::Instance()->GetSimTime();

  Param::Begin(&this->parameters);
  this->leftJointNameP = new ParamT<std::string>("leftJoint", "", 1);
  this->rightJointNameP = new ParamT<std::string>("rightJoint", "", 1);
  this->wheelSepP = new ParamT<float>("wheelSeparation", 0.34,1);
  this->wheelDiamP = new ParamT<float>("wheelDiameter", 0.15,1);
  this->torqueP = new ParamT<float>("torque", 10.0, 1);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->topicNameP = new ParamT<std::string>("topicName", "", 1);
  Param::End();

  this->x_ = 0;
  this->rot_ = 0;
  this->alive_ = true;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
DiffDrivePlugin::~DiffDrivePlugin()
{
  delete this->leftJointNameP;
  delete this->rightJointNameP;
  delete this->wheelSepP;
  delete this->wheelDiamP;
  delete this->torqueP;
  delete this->robotNamespaceP;
  delete this->topicNameP;
  delete this->callback_queue_thread_;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DiffDrivePlugin::LoadChild(XMLConfigNode *node)
{
  this->myIface = dynamic_cast<PositionIface*>(this->GetIface("position"));

  // the defaults are from pioneer2dx
  this->wheelSepP->Load(node);
  this->wheelDiamP->Load(node);
  this->torqueP->Load(node);

  this->leftJointNameP->Load(node);
  this->rightJointNameP->Load(node);

  this->joints[LEFT] = this->myParent->GetJoint(**this->leftJointNameP);
  this->joints[RIGHT] = this->myParent->GetJoint(**this->rightJointNameP);

  if (!this->joints[LEFT])
    gzthrow("The controller couldn't get left hinge joint");

  if (!this->joints[RIGHT])
    gzthrow("The controller couldn't get right hinge joint");

  // Initialize the ROS node and subscribe to cmd_vel

  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();

  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"diff_drive_plugin",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
    this->topicName, 1,
    boost::bind( &DiffDrivePlugin::cmdVelCallback, this, _1),
    ros::VoidPtr(), &this->queue_);
  this->sub_ = this->rosnode_->subscribe(so);  
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void DiffDrivePlugin::InitChild()
{
  // Reset odometric pose
  this->odomPose[0] = 0.0;
  this->odomPose[1] = 0.0;
  this->odomPose[2] = 0.0;

  this->odomVel[0] = 0.0;
  this->odomVel[1] = 0.0;
  this->odomVel[2] = 0.0;

  this->callback_queue_thread_ = new boost::thread( boost::bind( &DiffDrivePlugin::QueueThread,this ) );
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void DiffDrivePlugin::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->leftJointNameP) << "\n";
  stream << prefix << *(this->rightJointNameP) << "\n";
  stream << prefix << *(this->torqueP) << "\n";
  stream << prefix << *(this->wheelDiamP) << "\n";
  stream << prefix << *(this->wheelSepP) << "\n";
}


////////////////////////////////////////////////////////////////////////////////
// Reset
void DiffDrivePlugin::ResetChild()
{
  // Reset odometric pose
  this->odomPose[0] = 0.0;
  this->odomPose[1] = 0.0;
  this->odomPose[2] = 0.0;

  this->odomVel[0] = 0.0;
  this->odomVel[1] = 0.0;
  this->odomVel[2] = 0.0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void DiffDrivePlugin::UpdateChild()
{
  // TODO: Step should be in a parameter of this function
  double wd, ws;
  double d1, d2;
  double dr, da;
  Time stepTime;

  //this->myIface->Lock(1);

  this->GetPositionCmd();

  wd = **(this->wheelDiamP);
  ws = **(this->wheelSepP);


  //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
  stepTime = Simulator::Instance()->GetSimTime() - this->prevUpdateTime;
  this->prevUpdateTime = Simulator::Instance()->GetSimTime();

  // Distance travelled by front wheels
  d1 = stepTime.Double() * wd / 2 * this->joints[LEFT]->GetVelocity(0);
  d2 = stepTime.Double() * wd / 2 * this->joints[RIGHT]->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / ws;

  // Compute odometric pose
  this->odomPose[0] += dr * cos( this->odomPose[2] );
  this->odomPose[1] += dr * sin( this->odomPose[2] );
  this->odomPose[2] += da;

  // Compute odometric instantaneous velocity
  this->odomVel[0] = dr / stepTime.Double();
  this->odomVel[1] = 0.0;
  this->odomVel[2] = da / stepTime.Double();

  if (this->enableMotors)
  {
    this->joints[LEFT]->SetVelocity( 0, this->wheelSpeed[LEFT] /  
                                        (**(this->wheelDiamP) / 2.0) );

    this->joints[RIGHT]->SetVelocity( 0, this->wheelSpeed[RIGHT] /  
                                         (**(this->wheelDiamP) / 2.0) );

    this->joints[LEFT]->SetMaxForce( 0, **(this->torqueP) );
    this->joints[RIGHT]->SetMaxForce( 0, **(this->torqueP) );
  }

  //this->PutPositionData();

  //this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void DiffDrivePlugin::FiniChild()
{
  std::cout << "ENTERING FINALIZE\n";
  this->alive_ = false;
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_->join();
  std::cout << "EXITING FINALIZE\n";
}

// NEW: Now uses the target velocities from the ROS message, not the Iface 
void DiffDrivePlugin::GetPositionCmd()
{
  this->lock.lock();

  double vr, va;

  vr = this->x_; //this->myIface->data->cmdVelocity.pos.x;
  va = this->rot_; //this->myIface->data->cmdVelocity.yaw;

  //std::cout << "X: [" << this->x_ << "] ROT: [" << this->rot_ << "]" << std::endl;

  // Changed motors to be always on, which is probably what we want anyway
  this->enableMotors = true; //this->myIface->data->cmdEnableMotors > 0;

  //std::cout << this->enableMotors << std::endl;

  this->wheelSpeed[LEFT] = vr + va * **(this->wheelSepP) / 2;
  this->wheelSpeed[RIGHT] = vr - va * **(this->wheelSepP) / 2;

  this->lock.unlock();
}

// NEW: Store the velocities from the ROS message
void DiffDrivePlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
    //std::cout << "BEGIN CALLBACK\n";

    this->lock.lock();
    
    this->x_ = cmd_msg->linear.x;
    this->rot_ = cmd_msg->angular.z;
    
    this->lock.unlock();

    //std::cout << "END CALLBACK\n";
}

// NEW: custom callback queue thread
void DiffDrivePlugin::QueueThread()
{
  static const double timeout = 0.01;

  while (this->alive_ && this->rosnode_->ok())
  {
//    std::cout << "CALLING STUFF\n";
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}


// Update the data in the interface
// TODO: Update this to publish odometry topic
void DiffDrivePlugin::PutPositionData()
{
  // TODO: Data timestamp
  this->myIface->data->head.time = Simulator::Instance()->GetSimTime().Double();

  this->myIface->data->pose.pos.x = this->odomPose[0];
  this->myIface->data->pose.pos.y = this->odomPose[1];
  this->myIface->data->pose.yaw = NORMALIZE(this->odomPose[2]);

  this->myIface->data->velocity.pos.x = this->odomVel[0];
  this->myIface->data->velocity.yaw = this->odomVel[2];

  // TODO
  this->myIface->data->stall = 0;
}
