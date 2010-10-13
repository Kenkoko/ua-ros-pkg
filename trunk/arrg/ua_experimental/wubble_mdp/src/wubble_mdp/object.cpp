/*
 * object.cpp
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
 */

#include <boost/lexical_cast.hpp>
#include <boost/assign/std/vector.hpp>

#include <wubble_mdp/object.h>

using namespace std;
using namespace boost::assign;
using namespace wubble_mdp;
using boost::lexical_cast;

Object::Object(simulator_state::ObjectInfo obj_info)
{
  name_ = obj_info.name;
  x_ = obj_info.pose.position.x;
  y_ = obj_info.pose.position.y;
  orientation_ = extractYaw(obj_info);
}

Object::Object(oomdp_msgs::MDPObjectState state)
{
  name_ = state.name;
  map<string, string> attr_map = wubble_mdp::extractAttributeValues(state);
  x_ = lexical_cast<double> (attr_map["x"]);
  y_ = lexical_cast<double> (attr_map["y"]);
  orientation_ = wubble_mdp::convertOrientationString(attr_map["orientation"]);

  has_last_state_ = (attr_map["last_x"] == "None");
  if (has_last_state_)
  {
    last_x_ = lexical_cast<double> (attr_map["last_x"]);
    last_y_ = lexical_cast<double> (attr_map["last_y"]);
    last_orientation_ = wubble_mdp::convertOrientationString(attr_map["last_orientation"]);
  }
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
  x_ = new_info.pose.position.x;
  y_ = new_info.pose.position.y;
  orientation_ = extractYaw(new_info);
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

vector<oomdp_msgs::Relation> Object::computeBinaryRelations(Entity* other)
{
  vector<oomdp_msgs::Relation> relations;
  return relations;
}

oomdp_msgs::MDPObjectState Object::makeObjectState()
{
  oomdp_msgs::MDPObjectState state;
  state.class_name = this->getClassString();
  state.name = name_;
  state.attributes += "x", "y", "orientation", "last_x", "last_y", "last_orientation";
  state.values += doubleToString(x_), doubleToString(y_), makeOrientationString(orientation_);

  if (has_last_state_)
  {
    state .values += doubleToString(last_x_), doubleToString(last_y_), makeOrientationString(last_orientation_);
  }
  else
  {
    state.values += "None", "None", "None";
  }

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

