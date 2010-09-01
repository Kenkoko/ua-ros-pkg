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
 * Also computes various features such as the true distance (GJK) between objects.
 *
 * Author: Daniel Hewlett
 */

#include <algorithm>
#include <assert.h>

#include <simulator_experiments/world_state_publisher.h>

#include <gazebo/Geom.hh>
#include <gazebo/BoxShape.hh>
#include <gazebo/SphereShape.hh>
#include <gazebo/CylinderShape.hh>
#include <gazebo/PlaneShape.hh>
#include <gazebo/TrimeshShape.hh>
#include <gazebo/OgreVisual.hh>
#include <gazebo/OgreCreator.hh>
#include <gazebo/MeshManager.hh>
#include <gazebo/Mesh.hh>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletCollision/CollisionShapes/btBoxShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>
#include <BulletCollision/CollisionShapes/btCylinderShape.h>
#include <BulletCollision/CollisionShapes/btBox2dShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>

#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>

#include <OGRE/OgreMesh.h>

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
  this->blacklist_.push_back("clock");
  this->blacklist_.push_back("clock_body");
  this->blacklist_.push_back("gplane");
  this->blacklist_.push_back("plane");
  this->blacklist_.push_back("point_white");
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

btConvexShape* WorldStatePublisher::extract_shape(Geom* geom)
{
  // If we have cached a shape for this Geom, just return it
  map<Geom*, btConvexShape*>::iterator it = bt_shape_cache_.find(geom);
  if (it != bt_shape_cache_.end())
  {
    //    cout << "FOUND CACHED SHAPE! SWEET!" << endl;
    return it->second;
  }

  // Otherwise, create the appropriate shape depending on the object's type
  btConvexShape* result = NULL;
  switch (geom->GetShapeType())
  {
    case Shape::BOX:
    {
      BoxShape* box = dynamic_cast<BoxShape*> (geom->GetShape());

      ParamT<Vector3>* sizeP = dynamic_cast<ParamT<Vector3>*> (box->GetParam("size"));
      btVector3 size(abs(sizeP->GetValue().x), abs(sizeP->GetValue().y), abs(sizeP->GetValue().z));
      size *= 0.5;

      btBoxShape* bt_box = new btBoxShape(size);
      result = bt_box;

      break;
    }
    case Shape::SPHERE:
    {
      SphereShape* sphere = dynamic_cast<SphereShape*> (geom->GetShape());

      ParamT<double>* sizeP = dynamic_cast<ParamT<double>*> (sphere->GetParam("size"));

      btSphereShape* bt_sphere = new btSphereShape(sizeP->GetValue());
      result = bt_sphere;

      break;
    }
    case Shape::CYLINDER:
    {
      CylinderShape* cylinder = dynamic_cast<CylinderShape*> (geom->GetShape());

      ParamT<Vector2<double> >* sizeP = dynamic_cast<ParamT<Vector2<double> >*> (cylinder->GetParam("size"));
      double radius = sizeP->GetValue().x;
      double height = sizeP->GetValue().y;

      btVector3 size(radius, radius, height);

      btCylinderShapeZ* bt_cylinder = new btCylinderShapeZ(size);
      result = bt_cylinder;

      break;
    }
    case Shape::PLANE:
    {
      PlaneShape* plane = dynamic_cast<PlaneShape*> (geom->GetShape());

      ParamT<Vector2<double> >* sizeP = dynamic_cast<ParamT<Vector2<double> >*> (plane->GetParam("size"));
      double x = sizeP->GetValue().x * 0.5;
      double y = sizeP->GetValue().y * 0.5;

      btVector3 size(x, y, 0);

      btBox2dShape* bt_plane = new btBox2dShape(size);

      result = bt_plane;

      break;
    }
    case Shape::TRIMESH:
    {
      // Tried approximating with bounding boxes, and with btConvexHull, nothing seems to work

      TrimeshShape* trimesh = dynamic_cast<TrimeshShape*> (geom->GetShape());

      ParamT<string>* mesh_name = dynamic_cast<ParamT<string>*> (trimesh->GetParam("mesh"));
      const Mesh* mesh = MeshManager::Instance()->GetMesh(mesh_name->GetValue());
      const SubMesh* sub_mesh = mesh->GetSubMesh(0); // TODO: Could be more than 1 submesh, need to loop

      btConvexHullShape* convex_hull = new btConvexHullShape();

      cout << geom->GetName() << endl;
      cout << "==========================================" << endl;

      double xmin = 0, ymin = 0, zmin = 0;
      double xmax = 0, ymax = 0, zmax = 0;
      for (uint i = 0; i < sub_mesh->GetVertexCount(); i++)
      {
        Vector3 point_g = sub_mesh->GetVertex(i);
        btVector3 point_bt(point_g.x, point_g.y, point_g.z);

        if (point_g.x > xmax)
          xmax = point_g.x;
        if (point_g.y > ymax)
          ymax = point_g.y;
        if (point_g.z > zmax)
          zmax = point_g.z;
        if (point_g.x < xmin)
          xmin = point_g.x;
        if (point_g.y < ymin)
          ymin = point_g.y;
        if (point_g.z < zmin)
          zmin = point_g.z;

        convex_hull->addPoint(point_bt);
      }

      //      cout << xmin << " " << ymin << " " << zmin << endl;
      //      cout << xmax << " " << ymax << " " << zmax << endl;
      //      result = convex_hull;

      btVector3 size(xmax - xmin, ymax - ymin, zmax - zmin);
      //      size *= 0.5;
      result = new btBoxShape(size);

      cout << geom->GetWorldPose() << endl;
      print_vector(size);

      bt_shape_cache_[geom] = result;

      break;
    }
    default:
    {
      cerr << "CAN'T HANDLE SHAPE TYPE: " << geom->GetShapeType() << " RETURNING NULL" << endl;
      break;
    }
  }

  return result;
}

btTransform WorldStatePublisher::convert_transform(Pose3d pose)
{
  btVector3 pos(pose.pos.x, pose.pos.y, pose.pos.z);

  //  cout << "POS: " << pose.pos << endl;

  // This is right
  //  btQuaternion rot(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.u);
  btQuaternion rot(0, 0, 0, 1); // This is for AABBs

  //  cout << "QUATERNIONS!" << endl;
  //  cout << pose.rot.x << " " << pose.rot.y << " " << pose.rot.z << " " << pose.rot.u << endl;
  //  cout << pose.rot.GetAsEuler() << endl;
  //  cout << rot.getX() << " " << rot.getY() << " " << rot.getZ() << " " << rot.getW() << endl;
  //  btQuaternion test;
  //  test.setRPY(pose.rot.GetAsEuler().x, pose.rot.GetAsEuler().y, pose.rot.GetAsEuler().z);
  //  cout << test.getX() << " " << test.getY() << " " << test.getZ() << " " << test.getW() << endl;

  return btTransform(rot, pos);
}

btConvexShape* WorldStatePublisher::convert_aabb(Vector3 min, Vector3 max)
{
  Vector3 size = max - min;
  size /= 2;
  btVector3 half_extents(size.x, size.y, size.z);
  btConvexShape* box = new btBoxShape(half_extents);
  return box;
}

void WorldStatePublisher::UpdateChild()
{
  // Return if there's no one subscribing
  // TODO: Turn this back on
  //  if (this->worldStateConnectCount == 0)
  //    return;

  // Publish
  Time cur_time = Simulator::Instance()->GetSimTime();

  vector<Model*> models;
  vector<Model*>::iterator miter;
  vector<Body*> all_bodies;

  models = World::Instance()->GetModels();

  // GJK STUFF
  vector<btConvexShape*> boxes;
  vector<btTransform> tr;
  vector<string> gjk_names;
  //  vector<Model*> geom_to_model;

  // aggregate all bodies into a single vector
  for (miter = models.begin(); miter != models.end(); miter++)
  {
    // Let's try GJK with model bounding boxes now, at least that's something
    Model* model = *miter;
    string name = model->GetName();
    vector<string>::iterator result = find(blacklist_.begin(), blacklist_.end(), name);
    if (result == blacklist_.end())
    {
      Vector3 min, max;
      model->GetBoundingBox(min, max);
      boxes.push_back(convert_aabb(min, max));
      tr.push_back(convert_transform(model->GetWorldPose()));
      gjk_names.push_back(model->GetName());
    }

    const vector<Entity*> entities = (*miter)->GetChildren();
    // Iterate through all bodies
    vector<Entity*>::const_iterator eiter;
    for (eiter = entities.begin(); eiter != entities.end(); eiter++)
    {
      Body* body = dynamic_cast<Body*> (*eiter);
      if (body)
      {
        all_bodies.push_back(body);
      }
    }
  }

  // construct world state message
  if (!models.empty())
  {
    int count = 0;
    this->lock.lock();

    // compose worldStateMsg
    this->worldStateMsg.header.frame_id = this->frameName;
    this->worldStateMsg.header.stamp.fromSec(cur_time.Double());

    // Clear out the old list of objects
    this->worldStateMsg.object_info.clear();
    this->worldStateMsg.relations.clear();

    // Iterate through all_bodies
    vector<Model*>::iterator miter;
    for (miter = models.begin(); miter != models.end(); miter++)
    {
      Model* model = *miter;
      Body* body = model->GetCanonicalBody();

      string body_name = string(body->GetName());
      string model_name = string(model->GetName());

      vector<string>::iterator result = find(blacklist_.begin(), blacklist_.end(), model_name);
      if (result == blacklist_.end())
      {
        simulator_experiments::ObjectInfo info;

        info.name = model_name;

        // Let's try to look at the geoms
        for (map<string, Geom*>::const_iterator it = body->GetGeoms()->begin(); it != body->GetGeoms()->end(); it++)
        {
          Geom* geom = it->second;
          geom->SetContactsEnabled(true);

          //          boxes.push_back(extract_shape(geom));
          //          tr.push_back(convert_transform(geom->GetBody()->GetWorldPose()));
          //          gjk_names.push_back(geom->GetModel()->GetName() + "::" + geom->GetName());
          //          geom_to_model.push_back(geom->GetModel());
        }

        // set pose
        // get pose from simulator
        Pose3d pose;
        Quatern rot;
        Vector3 pos;
        // Get Pose/Orientation
        pose = body->GetWorldPose();
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
        Vector3 vpos = body->GetWorldLinearVel(); // get velocity in gazebo frame
        Vector3 veul = body->GetWorldAngularVel(); // get velocity in gazebo frame

        // pass linear rates
        info.velocity.linear.x = vpos.x;
        info.velocity.linear.y = vpos.y;
        info.velocity.linear.z = vpos.z;
        // pass euler angular rates
        info.velocity.angular.x = veul.x;
        info.velocity.angular.y = veul.y;
        info.velocity.angular.z = veul.z;

        // get forces
        Vector3 force = body->GetWorldForce(); // get velocity in gazebo frame
        Vector3 torque = body->GetWorldTorque(); // get velocity in gazebo frame
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

    // TODO: Need to add properties now to get some predicates

    // GJK Distance Calculation
    if (boxes.size() > 1)
    {
      for (uint i = 0; i < boxes.size() - 1; i++)
      {
        for (uint j = i + 1; j < boxes.size(); j++)
        {
          //          cout << "OBJECT " << i << ": " << gjk_names[i] << endl;
          //          cout << "OBJECT " << j << ": " << gjk_names[j] << endl;

          btVoronoiSimplexSolver gjk_simplex_solver;
          btGjkPairDetector gjk(boxes[0], boxes[1], &gjk_simplex_solver, NULL); // TODO: Add penetration solver.
          btPointCollector gjkOutput;
          btGjkPairDetector::ClosestPointInput input;

          input.m_transformA = tr[i];
          input.m_transformB = tr[j];

          gjk.getClosestPoints(input, gjkOutput, 0);

          //          cout << "DISTANCES:" << endl;
          double gjk_distance = 0.0;
          if (gjkOutput.m_hasResult)
          {
            if (gjkOutput.m_distance > 0)
            {
              //              cout << "CoM: " << tr[i].getOrigin().distance(tr[j].getOrigin()) << endl;
              //              cout << "GJK: " << gjkOutput.m_distance << endl;
              gjk_distance = gjkOutput.m_distance;
            }
            else
            {
              //cout << "SLIGHT COLLISION" << endl;
            }
          }
          else
          {
            //cout << "DEEP COLLISION" << endl;
          }

          simulator_experiments::Relation gjk_relation;
          gjk_relation.name = "GJK-Distance";
          gjk_relation.subject = gjk_names[i];
          gjk_relation.object = gjk_names[j];
          gjk_relation.is_symmetric = 1;
          gjk_relation.is_numeric = 1;
          gjk_relation.value = gjk_distance;

          // This is really for testing, we could compute this on the other end
          simulator_experiments::Relation contact_relation;
          contact_relation.name = "Contact";
          contact_relation.subject = gjk_names[i];
          contact_relation.object = gjk_names[j];
          contact_relation.is_symmetric = 1;
          contact_relation.is_numeric = 0;
          contact_relation.value = (gjk_distance == 0);

//          cout << gjk_relation << endl;

          this->worldStateMsg.relations.push_back(gjk_relation);
          this->worldStateMsg.relations.push_back(contact_relation);
        }
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

