/*
 * location.cpp
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
 */

#include <boost/lexical_cast.hpp>
#include <boost/assign/std/vector.hpp>

#include <wubble_mdp/location.h>

using namespace std;
using namespace boost::assign;
using boost::lexical_cast;

Location::Location(simulator_state::ObjectInfo obj_info)
{
  name_ = obj_info.name;
  x_ = obj_info.pose.position.x;
  y_ = obj_info.pose.position.y;
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
