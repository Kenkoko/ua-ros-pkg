/*
 * object.cpp
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
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

#include <wubble_mdp/object.h>
#include <wubble_mdp/relations.h>

using namespace std;
using namespace boost::assign;
using namespace wubble_mdp;
using boost::lexical_cast;
using boost::regex;
using boost::regex_replace;

Object::Object(simulator_state::ObjectInfo obj_info)
{
  name_ = obj_info.name;
  x_ = roundToDelta(obj_info.pose.position.x, delta_);
  y_ = roundToDelta(obj_info.pose.position.y, delta_);
  orientation_ = roundYaw(wubble_mdp::extractYaw(obj_info));

  // This is what will happen the first time an object is created, so there is no "last state"
  last_x_ = x_;
  last_y_ = y_;
  last_orientation_ = orientation_;

  ROS_INFO("HERE");

  // TODO: This method won't work for spawning unless we change a lot of things
}

Object::Object(oomdp_msgs::MDPObjectState state)
{
  ROS_INFO_STREAM(state);

  name_ = state.name;
  map<string, string> attr_map = wubble_mdp::extractAttributeValues(state);
  x_ = lexical_cast<double> (attr_map["x"]);
  y_ = lexical_cast<double> (attr_map["y"]);
  orientation_ = wubble_mdp::convertOrientationString(attr_map["orientation"]);

  last_x_ = lexical_cast<double> (attr_map["last_x"]);
  last_y_ = lexical_cast<double> (attr_map["last_y"]);
  last_orientation_ = wubble_mdp::convertOrientationString(attr_map["last_orientation"]);

  // These attributes never change, but are essential to define the object and create it
  size_x_ = lexical_cast<double> (attr_map["size_x"]);
  size_y_ = lexical_cast<double> (attr_map["size_y"]);
  size_z_ = lexical_cast<double> (attr_map["size_z"]);
  mass_ = lexical_cast<double> (attr_map["mass"]);
  static_ = lexical_cast<bool> (attr_map["static"]);
  shape_ = attr_map["shape"];
  color_ = attr_map["color"];
}

Object::~Object()
{
}

void Object::update(simulator_state::ObjectInfo new_info)
{
  // Copy the old state
  last_x_ = x_;
  last_y_ = y_;
  last_orientation_ = orientation_;

  // Update the state
  x_ = roundToDelta(new_info.pose.position.x, delta_);
  y_ = roundToDelta(new_info.pose.position.y, delta_);
  orientation_ = roundYaw(wubble_mdp::extractYaw(new_info));
}

// TODO: Only compute the XML if necessary
// TODO: Some of this is not necessary any more
bool Object::addToWorld()
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

    string simsem_path = ros::package::getPath("simulation_semantics") + "/objects/generic_object.xml";
    string file_contents;
    ifstream object_file(simsem_path.c_str());
    string line;
    if (object_file.is_open())
    {
      while (object_file.good())
      {
        getline(object_file, line);
        file_contents += line;
      }
      object_file.close();
    }

    // Gazebo Attributes
    // XYZ RPY NAME SHAPE STATIC SIZE MASS COLOR
    file_contents = regex_replace(file_contents, regex("NAME"), name_);
    file_contents = regex_replace(file_contents, regex("XYZ"), "0.0 0.0 " + doubleToString(size_z_ / 2));
    file_contents = regex_replace(file_contents, regex("RPY"), "0.0 0.0 0.0");
    file_contents = regex_replace(file_contents, regex("SHAPE"), shape_);
    file_contents = regex_replace(file_contents, regex("STATIC"), (static_ ? "true" : "false"));
    file_contents = regex_replace(file_contents, regex("SIZE"), doubleToString(size_x_) + " " + doubleToString(size_y_)
        + " " + doubleToString(size_z_));
    file_contents = regex_replace(file_contents, regex("MASS"), doubleToString(mass_));
    file_contents = regex_replace(file_contents, regex("COLOR"), color_);

    //  ROS_INFO_STREAM(file_contents);
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

std::string Object::getClassString()
{
  return "Object";
}

std::vector<oomdp_msgs::Relation> Object::computePredicates()
{
  vector<oomdp_msgs::Relation> result;

  // TODO: What predicates do objects have? Moved?

  return result;
}

vector<oomdp_msgs::Relation> Object::computeBinaryRelations(EntityPtr other)
{
  vector<oomdp_msgs::Relation> relations;
  return relations;
}

oomdp_msgs::MDPObjectState Object::makeObjectState()
{
  oomdp_msgs::MDPObjectState state;
  state.class_name = this->getClassString();
  state.name = name_;
  state.attributes += "x", "y", "orientation",
                      "last_x", "last_y", "last_orientation",
                      "size_x", "size_y", "size_z",
                      "mass", "static", "shape", "color";
  state.values += doubleToString(x_), doubleToString(y_), makeOrientationString(orientation_);
  state.values += doubleToString(last_x_), doubleToString(last_y_), makeOrientationString(last_orientation_);
  state.values += doubleToString(size_x_), doubleToString(size_y_), doubleToString(size_z_);
  state.values += doubleToString(mass_);
  state.values += (static_ ? "1" : "0");
  state.values += shape_;
  state.values += color_;
  return state;
}

btVector3 Object::getPosition()
{
  return btVector3(x_, y_, 0.0);
}

btVector3 Object::getLastPosition()
{
  return btVector3(last_x_, last_y_, 0.0);
}

geometry_msgs::Pose Object::getPose()
{
  geometry_msgs::Pose pose;
  pose.position.x = x_;
  pose.position.y = y_;
  btQuaternion q;
  q.setRPY(0, 0, orientation_);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

