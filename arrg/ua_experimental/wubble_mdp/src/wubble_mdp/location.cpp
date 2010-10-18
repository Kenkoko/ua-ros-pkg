/*
 * location.cpp
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
 */

#include <iostream>
#include <fstream>
#include <boost/lexical_cast.hpp>
#include <boost/assign/std/vector.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo/SpawnModel.h>

#include <wubble_mdp/location.h>
#include <wubble_mdp/robot.h>

using namespace std;
using namespace boost::assign;
using boost::lexical_cast;

Location::Location(simulator_state::ObjectInfo obj_info)
{
  name_ = obj_info.name;
  x_ = wubble_mdp::roundToDelta(obj_info.pose.position.x, Robot::delta_);
  y_ = wubble_mdp::roundToDelta(obj_info.pose.position.y, Robot::delta_);
}

Location::Location(oomdp_msgs::MDPObjectState state)
{
  name_ = state.name;
  map<string, string> attr_map = wubble_mdp::extractAttributeValues(state);
  x_ = lexical_cast<double> (attr_map["x"]);
  y_ = lexical_cast<double> (attr_map["y"]);
}

Location::~Location()
{
}

oomdp_msgs::MDPObjectState Location::makeObjectState()
{
  oomdp_msgs::MDPObjectState state;
  state.class_name = this->getClassString();
  state.name = name_;
  state.attributes += "x", "y";
  state.values += wubble_mdp::doubleToString(x_), wubble_mdp::doubleToString(y_);

  return state;
}

bool Location::addToWorld()
{
  gazebo::SpawnModel spawn_model;
  spawn_model.request.model_name = name_;
  spawn_model.request.initial_pose = getPose();

  string simsem_path = ros::package::getPath("simulation_semantics") + "/objects/point.xml";
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

string Location::getClassString()
{
  return "Location";
}

void Location::update(simulator_state::ObjectInfo new_info)
{
  // TODO: Should this ever happen?
  x_ = new_info.pose.position.x;
  y_ = new_info.pose.position.y;
}

vector<oomdp_msgs::Relation> Location::computePredicates()
{
  // Location has no predicates
  vector<oomdp_msgs::Relation> result;
  return result;
}

vector<oomdp_msgs::Relation> Location::computeBinaryRelations(Entity* other)
{
  vector<oomdp_msgs::Relation> relations;
  return relations;
}

btVector3 Location::getPosition()
{
  return btVector3(x_, y_, 0.0);
}

btVector3 Location::getLastPosition()
{
  return btVector3(x_, y_, 0.0);
}

geometry_msgs::Pose Location::getPose()
{
  geometry_msgs::Pose pose;
  pose.position.x = x_;
  pose.position.y = y_;
  return pose;
}
