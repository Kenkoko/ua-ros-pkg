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
 * Desc: Publishes the world state from Gazebo in a friendly message
 * Author: Daniel Hewlett
 */

#include <algorithm>
#include <assert.h>

#include <simulator_experiments/world_state_publisher.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("world_state_publisher", WorldStatePublisher)
;

// Constructor
WorldStatePublisher::WorldStatePublisher(Entity *parent) :
  Controller(parent)
{
  this->parent_model_ = dynamic_cast<Model*> (this->parent);

  if (!this->parent_model_)
    gzthrow("GazeboMechanismControl controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<string> ("robotNamespace", "/", 0);
  this->topicNameP = new ParamT<string> ("topicName", "", 1);
  this->frameNameP = new ParamT<string> ("frameName", "map", 0);
  Param::End();

  // TODO: Hardcoded for now, should probably be from a ROS param
  this->blacklist_.push_back("clock_body");
  this->blacklist_.push_back("plane");
  this->blacklist_.push_back("point_white_RenderableBody_0");

  this->worldStateConnectCount = 0;
}

// Destructor
WorldStatePublisher::~WorldStatePublisher()
{
  delete this->robotNamespaceP;
  delete this->topicNameP;
  delete this->frameNameP;
  delete this->rosnode_;
}

void WorldStatePublisher::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();

  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();
  this->frameNameP->Load(node);
  this->frameName = this->frameNameP->GetValue();

  // Custom Callback Queue
  ros::AdvertiseOptions
                        ao =
                            ros::AdvertiseOptions::create<simulator_experiments::WorldState>(
                                                                                             this->topicName,
                                                                                             1,
                                                                                             boost::bind(
                                                                                                         &WorldStatePublisher::WorldStateConnect,
                                                                                                         this),
                                                                                             boost::bind(
                                                                                                         &WorldStatePublisher::WorldStateDisconnect,
                                                                                                         this),
                                                                                             ros::VoidPtr(),
                                                                                             &this->queue_);
  this->pub_ = this->rosnode_->advertise(ao);
}

// Someone subscribes to me
void WorldStatePublisher::WorldStateConnect()
{
  this->worldStateConnectCount++;
}

// Someone unsubscribes from me
void WorldStatePublisher::WorldStateDisconnect()
{
  this->worldStateConnectCount--;
}

void WorldStatePublisher::InitChild()
{
  // Custom Callback Queue
  this->callback_queue_thread_ = new boost::thread(boost::bind(&WorldStatePublisher::QueueThread, this));
}

void WorldStatePublisher::UpdateChild()
{
  // Return if there's no one subscribing
  if (this->worldStateConnectCount == 0)
    return;

  // Publish
  Time cur_time = Simulator::Instance()->GetSimTime();


//  World::Instance()->GetPhysicsEngine()->



  vector<gazebo::Model*> models;
  vector<gazebo::Model*>::iterator miter;

  map<string, gazebo::Body*> all_bodies;
  all_bodies.clear();

  models = gazebo::World::Instance()->GetModels();

  // aggregate all bodies into a single vector
  for (miter = models.begin(); miter != models.end(); miter++)
  {
//    BOOST_FOREACH(Entity* e, miter->GetChildren()) {
//      miter->GetBody(e->GetName());
//    }

    // TODO: For now, we can only get the default body
    Body *body = (*miter)->GetBody();

    all_bodies.insert(make_pair(body->GetName(), body));
  }
  //ROS_ERROR("debug: %d",all_bodies.size());

  // construct world state message
  if (!all_bodies.empty())
  {
    int count = 0;
    this->lock.lock();

    // compose worldStateMsg
    this->worldStateMsg.header.frame_id = this->frameName;
    this->worldStateMsg.header.stamp.fromSec(cur_time.Double());

    // Clear out the old list of objects
    this->worldStateMsg.object_info.clear();

    // Iterate through all_bodies
    map<string, Body*>::iterator biter;
    for (biter = all_bodies.begin(); biter != all_bodies.end(); biter++)
    {
      string name = string(biter->second->GetName());

      vector<string>::iterator result = find(blacklist_.begin(), blacklist_.end(), name);
      if (result == blacklist_.end())
      {
        simulator_experiments::ObjectInfo info;

        info.name = name;

        // set pose
        // get pose from simulator
        Pose3d pose;
        Quatern rot;
        Vector3 pos;
        // Get Pose/Orientation 
        pose = (biter->second)->GetAbsPose();
        pos = pose.pos;
        rot = pose.rot;

        info.pose.position.x = pos.x;
        info.pose.position.y = pos.y;
        info.pose.position.z = pos.z;
        info.pose.orientation.x = rot.x;
        info.pose.orientation.y = rot.y;
        info.pose.orientation.z = rot.z;
        info.pose.orientation.w = rot.u;

        // set velocities
        // get Rates
        Vector3 vpos = (biter->second)->GetPositionRate(); // get velocity in gazebo frame
        Quatern vrot = (biter->second)->GetRotationRate(); // get velocity in gazebo frame
        Vector3 veul = (biter->second)->GetEulerRate(); // get velocity in gazebo frame

        // pass linear rates
        info.velocity.linear.x = vpos.x;
        info.velocity.linear.y = vpos.y;
        info.velocity.linear.z = vpos.z;
        // pass euler angular rates
        info.velocity.angular.x = veul.x;
        info.velocity.angular.y = veul.y;
        info.velocity.angular.z = veul.z;

        // get forces
        Vector3 force = (biter->second)->GetForce(); // get velocity in gazebo frame
        Vector3 torque = (biter->second)->GetTorque(); // get velocity in gazebo frame
        info.force.force.x = force.x;
        info.force.force.y = force.y;
        info.force.force.z = force.z;
        info.force.torque.x = torque.x;
        info.force.torque.y = torque.y;
        info.force.torque.z = torque.z;

        count++;

        this->worldStateMsg.object_info.push_back(info);
      }
    }
    this->pub_.publish(this->worldStateMsg);
    this->lock.unlock();

  }

}

void WorldStatePublisher::FiniChild()
{
  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_->join();
}

// custom callback queue thread
void WorldStatePublisher::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

