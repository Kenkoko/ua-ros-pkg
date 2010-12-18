/*
 * item.cpp
 *
 *  Created on: Dec 2, 2010
 *      Author: Daniel Hewlett
 */

#include <iostream>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/assign/std/vector.hpp>
#include <boost/regex.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/SpawnModel.h>
#include <gazebo/SetModelState.h>

#include <wubble_mdp/item.h>
#include <wubble_mdp/relations.h>
#include <wubble_mdp/robot.h>

using namespace std;
using namespace boost::assign;
using namespace wubble_mdp;
using boost::lexical_cast;
using boost::regex;
using boost::regex_replace;

// TODO: Do we need this?
Item::Item(simulator_state::ObjectInfo obj_info)
{
  cerr << "You didn't write this yet!" << endl;
}

Item::Item(oomdp_msgs::MDPObjectState state)
{
  name_ = state.name;
  map<string, string> attr_map = wubble_mdp::extractAttributeValues(state);
  x_ = lexical_cast<double> (attr_map["x"]);
  y_ = lexical_cast<double> (attr_map["y"]);

  last_x_ = lexical_cast<double> (attr_map["last_x"]);
  last_y_ = lexical_cast<double> (attr_map["last_y"]);
  is_carried_ = lexical_cast<bool> (attr_map["is_carried"]);
  carrier_name_ = attr_map["carrier_name"];
}

Item::~Item()
{
}

// TODO: carrier_name sort of makes is_carried redundant
oomdp_msgs::MDPObjectState Item::makeObjectState()
{
  oomdp_msgs::MDPObjectState state;
  state.class_name = this->getClassString();
  state.name = name_;
  state.attributes += "x", "y", "last_x", "last_y", "is_carried", "carrier_name";
  state.values += doubleToString(x_), doubleToString(y_);
  state.values += doubleToString(last_x_), doubleToString(last_y_);
  state.values += (is_carried_ ? "1" : "0");
  state.values += carrier_name_;
  return state;
}

void Item::update(simulator_state::ObjectInfo new_info)
{
  // Updates come through the robot, so this method does nothing
}

std::string Item::getClassString()
{
  return "Item";
}

btVector3 Item::getPosition()
{
  return btVector3(x_, y_, 0.0);
}

btVector3 Item::getLastPosition()
{
  return btVector3(last_x_, last_y_, 0.0);
}

geometry_msgs::Pose Item::getPose()
{
  geometry_msgs::Pose pose;
  pose.position.x = x_;
  pose.position.y = y_;
  btQuaternion q;
  q.setRPY(0, 0, 0);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

std::vector<oomdp_msgs::Relation> Item::computePredicates()
{
  vector<oomdp_msgs::Relation> relations;

  // TODO: Should also have Moved predicate

  vector<string> names;
  names.push_back(name_);
  relations.push_back(wubble_mdp::makeRelation("IsCarried", names, is_carried_));

  return relations;
}

std::vector<oomdp_msgs::Relation> Item::computeBinaryRelations(EntityPtr other)
{
  vector<oomdp_msgs::Relation> relations;

  // TODO: Should have distance/collison predicates

  return relations;
}

bool Item::addToWorld()
{
  if (wubble_mdp::existsInWorld(name_))
  {
    gazebo::SetModelState set_model_state;
    set_model_state.request.model_state.model_name = name_;
    set_model_state.request.model_state.pose = getPose();
    return ros::service::call("gazebo/set_model_state", set_model_state);
  }
  else
  {
    gazebo::SpawnModel spawn_model;
    spawn_model.request.model_name = name_;
    spawn_model.request.initial_pose = getPose();

    string simsem_path = ros::package::getPath("simulation_semantics") + "/objects/item.xml";
    string file_contents;
    ifstream point_file(simsem_path.c_str());
    string line;
    if (point_file.is_open())
    {
      while (point_file.good())
      {
        getline(point_file, line);
        file_contents += line;
      }
      point_file.close();
    }
    spawn_model.request.model_xml = file_contents;

    bool service_called = ros::service::call("gazebo/spawn_gazebo_model", spawn_model);
    if (service_called)
    {
      if (spawn_model.response.success)
      {
        return true;
      }
      else
      {
        ROS_ERROR_STREAM(spawn_model.response.status_message);
        return false;
      }
    }
    else
    {
      return false;
    }
  }
}

bool Item::shouldInitPickUp(Robot* robot)
{
  return (carrier_name_ != "NONE" && robot->name_ == carrier_name_);
}

// TODO: Later, add some stochasticity here to be more realistic
bool Item::handlePickUp(Robot* robot)
{
  // For now, must be at the object's location to pick it up
  double delta = 0.5;
  if (wubble_mdp::roundToDelta(robot->x_, delta) == wubble_mdp::roundToDelta(x_, delta)
      && wubble_mdp::roundToDelta(robot->y_, delta) == wubble_mdp::roundToDelta(y_, delta))
  {
    is_carried_ = true;
    carrier_name_ = robot->name_;
  }
  else
  {
    is_carried_ = false;
  }

  return is_carried_;
}

bool Item::handlePutDown(Robot* robot)
{
  is_carried_ = false;
  carrier_name_ = "NONE";
  return true; // Always succeeds for now
}

void Item::updatePosition(Robot* robot, bool move_in_world)
{
  if (!is_carried_) // Sanity check
  {
    cerr << "Robot is updating uncarried object" << endl;
  }
  else
  {
    last_x_ = x_;
    last_y_ = y_;
    x_ = robot->x_;
    y_ = robot->y_;
  }

  if (move_in_world)
  {
    addToWorld(); // Hopefully this is safe
  }
}
