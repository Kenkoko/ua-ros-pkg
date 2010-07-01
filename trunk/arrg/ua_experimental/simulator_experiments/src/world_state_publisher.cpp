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

#include <gazebo/Geom.hh>
#include <gazebo/BoxShape.hh>
#include <gazebo/SphereShape.hh>
#include <gazebo/CylinderShape.hh>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>

using namespace std;
using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("world_state_publisher", WorldStatePublisher)
;

//static btVoronoiSimplexSolver sGjkSimplexSolver;
//btSimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;

void print_vector(btVector3 v)
{
  cout << v.x() << " " << v.y() << " " << v.z() << endl;
}

void print_quat(btQuaternion q)
{
  cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
}

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

btVector3 WorldStatePublisher::extract_size(Shape* shape)
{
  ParamT<Vector3>* size = dynamic_cast<ParamT<Vector3>*> (shape->GetParam("size"));

  // FOR SOME REASON, WE ARE SOMETIMES GETTING NEGATIVE SIZES?
  return btVector3(abs(size->GetValue().x), abs(size->GetValue().y), abs(size->GetValue().z));
}

btTransform WorldStatePublisher::convert_transform(Pose3d pose)
{
  btVector3 pos(pose.pos.x, pose.pos.y, pose.pos.z);
  btQuaternion rot(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.u);
  return btTransform(rot, pos);
}

void WorldStatePublisher::UpdateChild()
{
  // Return if there's no one subscribing
  if (this->worldStateConnectCount == 0)
    return;

  // Publish
  Time cur_time = Simulator::Instance()->GetSimTime();

  vector<gazebo::Model*> models;
  vector<gazebo::Model*>::iterator miter;
  vector<gazebo::Body*> all_bodies;

  models = gazebo::World::Instance()->GetModels();

  // aggregate all bodies into a single vector
  for (miter = models.begin(); miter != models.end(); miter++)
  {
    const std::vector<gazebo::Entity*> entities = (*miter)->GetChildren();
    // Iterate through all bodies
    std::vector<Entity*>::const_iterator eiter;
    for (eiter = entities.begin(); eiter != entities.end(); eiter++)
    {
      gazebo::Body* body = dynamic_cast<gazebo::Body*> (*eiter);
      if (body)
      {
        all_bodies.push_back(body);
      }
    }
  }

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

    // This is where we store the shapes and their transforms for GJK
    vector<btConvexShape*> boxes;
    vector<btTransform> tr;

    // Iterate through all_bodies
    vector<Body*>::iterator biter;
    for (biter = all_bodies.begin(); biter != all_bodies.end(); biter++)
    {
      Body* body = *biter; //->second;

      string name = string(body->GetName());

      vector<string>::iterator result = find(blacklist_.begin(), blacklist_.end(), name);
      if (result == blacklist_.end())
      {
        simulator_experiments::ObjectInfo info;

        info.name = body->GetModel()->GetName();

        // Let's try to look at the geoms
        for (map<string, Geom*>::const_iterator it = body->GetGeoms()->begin(); it != body->GetGeoms()->end(); it++)
        {
          Geom* geom = it->second;
//          geom->SetContactsEnabled(true);

          switch (geom->GetShapeType())
          {
            case Shape::BOX:
            {
              BoxShape* box = dynamic_cast<BoxShape*> (geom->GetShape());
              btVector3 size = extract_size(box);
              size *= 0.5;

              btBoxShape* bt_box = new btBoxShape(size);
              boxes.push_back(bt_box);

              tr.push_back(convert_transform(geom->GetAbsPose()));
              break;
            }
            case Shape::SPHERE:
            {
              cout << "HERE1" << endl;
              SphereShape* sphere = dynamic_cast<SphereShape*> (geom->GetShape());
              cout << "HERE1.5" << endl;
              // TODO: This is breaking it for some reason
              // Probably the sphere has only one dimension of size, not 3?
              btVector3 size = extract_size(sphere);

              cout << "HERE2" << endl;
              btSphereShape* bt_sphere = new btSphereShape(size.x());
              boxes.push_back(bt_sphere);

              cout << "HERE3" << endl;
              tr.push_back(convert_transform(geom->GetAbsPose()));

              cout << "SPHERE: " << size.x() << " " << size.y() << " " << size.z() << endl;
              break;
            }
            default:
              cout << "CAN'T HANDLE SHAPE TYPE: " << geom->GetShapeType() << endl;
          }
        }

        // set pose
        // get pose from simulator
        Pose3d pose;
        Quatern rot;
        Vector3 pos;
        // Get Pose/Orientation
        pose = body->GetAbsPose();
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
        Vector3 vpos = body->GetPositionRate(); // get velocity in gazebo frame
        Quatern vrot = body->GetRotationRate(); // get velocity in gazebo frame
        Vector3 veul = body->GetEulerRate(); // get velocity in gazebo frame

        // pass linear rates
        info.velocity.linear.x = vpos.x;
        info.velocity.linear.y = vpos.y;
        info.velocity.linear.z = vpos.z;
        // pass euler angular rates
        info.velocity.angular.x = veul.x;
        info.velocity.angular.y = veul.y;
        info.velocity.angular.z = veul.z;

        // get forces
        Vector3 force = body->GetForce(); // get velocity in gazebo frame
        Vector3 torque = body->GetTorque(); // get velocity in gazebo frame
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

    if (boxes.size() > 1)
    {
      cout << "ATTEMPTING GJK" << endl;

      btVoronoiSimplexSolver sGjkSimplexSolver;
      //      btSimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;

      cout << "OBJECT 0: " << endl;
      //print_vector(boxes[0].getHalfExtentsWithoutMargin());
      //      print_vector(tr[0].getOrigin());
      //      print_quat(tr[0].getRotation());
      cout << "OBJECT 1: " << endl;
      //print_vector(boxes[1].getHalfExtentsWithoutMargin());
      //      print_vector(tr[1].getOrigin());
      //      print_quat(tr[1].getRotation());

      btGjkPairDetector gjk(boxes[0], boxes[1], &sGjkSimplexSolver, NULL);
      btPointCollector gjkOutput;
      btGjkPairDetector::ClosestPointInput input;

      input.m_transformA = tr[0];
      input.m_transformB = tr[1];

      cout << "DISTANCES:" << endl;

      gjk.getClosestPoints(input, gjkOutput, 0);

      if (gjkOutput.m_hasResult)
      {
        if (gjkOutput.m_distance > 0)
        {
          //        btVector3 endPt = gjkOutput.m_pointInWorld + gjkOutput.m_normalOnBInWorld * gjkOutput.m_distance;
          //        print_vector(gjkOutput.m_pointInWorld);
          //        print_vector(endPt);
          cout << "CoM: " << tr[0].getOrigin().distance(tr[1].getOrigin()) << endl;
          cout << "GJK: " << gjkOutput.m_distance << endl;
        }
        else
        {
          cout << "SLIGHT COLLISION" << endl;
        }
      }
      else
      {
        cout << "DEEP COLLISION" << endl;
      }
    }

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

