/*
 * object.cpp
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
 */

#include <boost/lexical_cast.hpp>
#include <boost/assign/std/vector.hpp>

#include <wubble_mdp/object.h>
#include <wubble_mdp/relations.h>

using namespace std;
using namespace boost::assign;
using namespace wubble_mdp;
using boost::lexical_cast;

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
}

Object::Object(oomdp_msgs::MDPObjectState state)
{
  name_ = state.name;
  map<string, string> attr_map = wubble_mdp::extractAttributeValues(state);
  x_ = lexical_cast<double> (attr_map["x"]);
  y_ = lexical_cast<double> (attr_map["y"]);
  orientation_ = wubble_mdp::convertOrientationString(attr_map["orientation"]);

  last_x_ = lexical_cast<double> (attr_map["last_x"]);
  last_y_ = lexical_cast<double> (attr_map["last_y"]);
  last_orientation_ = wubble_mdp::convertOrientationString(attr_map["last_orientation"]);
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
  state.values += wubble_mdp::doubleToString(x_), wubble_mdp::doubleToString(y_);
  state.values += wubble_mdp::makeOrientationString(orientation_);
  state.values += wubble_mdp::doubleToString(last_x_), wubble_mdp::doubleToString(last_y_);
  state.values += wubble_mdp::makeOrientationString(last_orientation_);
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

