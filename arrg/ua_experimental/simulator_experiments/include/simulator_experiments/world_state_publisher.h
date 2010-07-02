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
#ifndef WORLD_STATE_PUBLISHER_HH
#define WORLD_STATE_PUBLISHER_HH

#include <ros/ros.h>
// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/Model.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/Shape.hh>
#include <gazebo/Pose3d.hh>

#include <simulator_experiments/WorldState.h>
#include <simulator_experiments/ObjectInfo.h>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btTransform.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>

#include <map>
#include <vector>
#include <string>

#include <boost/thread.hpp>

namespace gazebo
{

class WorldStatePublisher : public Controller
{

public:
  WorldStatePublisher(Entity *parent);
  virtual ~WorldStatePublisher();

protected:
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:
  int worldStateConnectCount;
  void WorldStateConnect();
  void WorldStateDisconnect();

  btConvexShape* extract_shape(Geom* geom);
  btTransform convert_transform(gazebo::Pose3d pose);

  simulator_experiments::WorldState worldStateMsg;

  gazebo::Model* parent_model_;

  ros::NodeHandle* rosnode_;
  ros::Publisher pub_;

  boost::mutex lock;

  ParamT<std::string> *robotNamespaceP;
  std::string robotNamespace;
  ParamT<std::string> *topicNameP;
  std::string topicName;
  ParamT<std::string> *frameNameP;
  std::string frameName;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  void QueueThread();
  boost::thread* callback_queue_thread_;

  std::vector<std::string> blacklist_;

  std::map<gazebo::Geom*,btConvexShape*> bt_shape_cache_;
  btVoronoiSimplexSolver gjk_simplex_solver_;

};

}
#endif

