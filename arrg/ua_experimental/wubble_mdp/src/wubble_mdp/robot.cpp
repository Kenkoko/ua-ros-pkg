/*
 * robot.cpp
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
 */

#include <algorithm>
#include <math.h>
#include <boost/math/constants/constants.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/assign/std/vector.hpp>

#include <ros/ros.h>
#include <gazebo/GetWorldProperties.h>
#include <gazebo/SetModelState.h>
#include <wubble_description/SpawnWubbleBase.h>
#include <wubble_description/DeleteWubbleBase.h>

#include <wubble_mdp/robot.h>

using namespace std;
using namespace geometry_msgs;
using namespace boost::assign;
using boost::lexical_cast;
using oomdp_msgs::Relation;
using wubble_mdp::roundToDelta;
using wubble_mdp::roundYaw;

Robot::Robot(simulator_state::ObjectInfo obj_info)
{
  name_ = obj_info.name;
  x_ = roundToDelta(obj_info.pose.position.x, delta_);
  y_ = roundToDelta(obj_info.pose.position.y, delta_);
  orientation_ = wubble_mdp::convertOrientationString(wubble_mdp::extractOrientationString(obj_info));

  // This is what will happen the first time an object is created, so there is no "last state"
  last_x_ = x_;
  last_y_ = y_;
  last_orientation_ = orientation_;
}

Robot::Robot(oomdp_msgs::MDPObjectState state)
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

Robot::~Robot()
{
}

void Robot::update(simulator_state::ObjectInfo new_info)
{
  // Copy the old state
  last_x_ = x_;
  last_y_ = y_;
  last_orientation_ = orientation_;

  // Update the state
  x_ = roundToDelta(new_info.pose.position.x, delta_);
  y_ = roundToDelta(new_info.pose.position.y, delta_);
  orientation_ = wubble_mdp::convertOrientationString(wubble_mdp::extractOrientationString(new_info));
}

bool Robot::addToWorld()
{
  // If the robot is missing, spawn him
  if (wubble_mdp::existsInWorld(name_))
  {
    gazebo::SetModelState set_model_state;
    set_model_state.request.model_state.model_name = name_;
    set_model_state.request.model_state.pose = getPose();
    return ros::service::call("gazebo/set_model_state", set_model_state);
  }
  else // Otherwise, just move him
  {
    wubble_description::SpawnWubbleBase swb;
    swb.request.name = name_;
    swb.request.initial_pose = getPose();
    return ros::service::call("spawn_wubble_base", swb);
  }
}

bool Robot::removeFromWorld()
{
  wubble_description::DeleteWubbleBase dwb;
  dwb.request.name = name_;
  return ros::service::call("delete_wubble_base", dwb);
}

std::string Robot::getClassString()
{
  return "Robot";
}

oomdp_msgs::MDPObjectState Robot::makeObjectState()
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

vector<Relation> Robot::computePredicates()
{
  vector<Relation> result;

  vector<string> names;
  names.push_back(name_);

  bool forward = (x_ != last_x_ || y_ != last_y_);
  result.push_back(wubble_mdp::makeRelation("Forward", names, forward));
  bool turned = (orientation_ != last_orientation_);
  result.push_back(wubble_mdp::makeRelation("Turned", names, turned));
  result.push_back(wubble_mdp::makeRelation("Moved", names, (forward || turned)));
  if (turned)
  {
    double delta_angle = wubble_mdp::deltaAngle(orientation_, last_orientation_);
    bool left = (delta_angle > 0);
    result.push_back(wubble_mdp::makeRelation("Left", names, left));
    result.push_back(wubble_mdp::makeRelation("Right", names, !left));
  }
  else
  {
    result.push_back(wubble_mdp::makeRelation("Left", names, false));
    result.push_back(wubble_mdp::makeRelation("Right", names, false));
  }

  // TEMPORARILY DISABLED
  //  vector<string> directions;
  //  directions += "N", "NE", "E", "SE", "S", "SW", "W", "NW";
  //  string orientation_string = wubble_mdp::makeOrientationString(orientation_);
  //  for (vector<string>::iterator dit = directions.begin(); dit != directions.end(); ++dit)
  //  {
  //    result.push_back(wubble_mdp::makeRelation("Orientation" + orientation_string, names, (*dit == orientation_string)));
  //  }

  return result;
}

vector<Relation> Robot::computeBinaryRelations(Entity* other)
{
  const double pi = boost::math::constants::pi<double>();

  vector<Relation> relations;
  string rels[] = {"RightOf", "LeftOf", "InFrontOf", "Behind"};
  int true_index = -1;

  if (wubble_mdp::chessDistance(getPosition(), other->getPosition()) > 0)
  {
    double rel_angle = computeRelativeAngle(other->getPosition());

    true_index = 0;
    if (rel_angle < 0)
    {
      true_index = 1;
      rel_angle = abs(rel_angle);
    }

    if (rel_angle < (pi / 4) - 0.1)
    {
      true_index = 2;
    }
    else if (rel_angle > (3 * pi / 4) + 0.1)
    {
      true_index = 3;
    }
  }

  vector<string> names;
  names.push_back(name_);
  names.push_back(other->name_);
  for (int i = 0; i < 4; i++)
  {
    relations.push_back(wubble_mdp::makeRelation(rels[i], names, (true_index == i)));
  }

  return relations;
}

double Robot::computeRelativeAngle(btVector3 other_pos)
{
  btVector3 my_pos = getPosition();

  double x_diff = other_pos.x() - my_pos.x();
  double y_diff = other_pos.y() - my_pos.y();

  double direct_angle = atan2(y_diff, x_diff);

  return wubble_mdp::deltaAngle(orientation_, direct_angle);
}

btVector3 Robot::getPosition()
{
  return btVector3(x_, y_, 0.0);
}

btVector3 Robot::getLastPosition()
{
  return btVector3(last_x_, last_y_, 0.0);
}

void Robot::simulateAction(string action, vector<Object*> objects)
{
  // Copy the old state
  last_x_ = x_;
  last_y_ = y_;
  last_orientation_ = orientation_;

  btVector3 new_pose = computeNewPose(action);

  // This gives him virtual "walls" when planning
  bool is_valid = true;

  if (-5 > new_pose.x() || new_pose.x() > 5)
  {
    is_valid = false;
  }
  if (-5 > new_pose.y() || new_pose.y() > 5)
  {
    is_valid = false;
  }

  for (vector<Object*>::iterator obj_it = objects.begin(); obj_it != objects.end(); ++obj_it)
  {
    if (wubble_mdp::chessDistance(new_pose, (*obj_it)->getPosition()) == 0)
    {
      // Freeze if you're trying to go into an object (TODO: Later will need more sophisticated checks)
      is_valid = false;
      is_valid = false;
    }
  }

  if (is_valid)
  {
    x_ = new_pose.x();
    y_ = new_pose.y();
  }
  orientation_ = new_pose.z();
}

Pose Robot::computeTargetPose(string action)
{
  Pose pose;

  btVector3 new_pose = computeNewPose(action);
  //  cout << new_pose.getX() << "," << new_pose.getY() << "," << new_pose.getZ() << endl;
  pose.position.x = new_pose.x();
  pose.position.y = new_pose.y();
  btQuaternion q;
  q.setRPY(0, 0, new_pose.z());
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return pose;
}

// Return (x, y, orientation)
btVector3 Robot::computeNewPose(string action)
{
  //  cout << "OLD " << x_ << " " << y_ << " " << orientation_ << endl;

  btVector3 new_pose(x_, y_, orientation_);

  if (action == "forward")
  {
    string orientation = wubble_mdp::makeOrientationString(orientation_);
    if (orientation.find("N") != string::npos)
    {
      new_pose.setY(new_pose.y() + delta_);
    }
    else if (orientation.find("S") != string::npos)
    {
      new_pose.setY(new_pose.y() - delta_);
    }

    if (orientation.find("E") != string::npos)
    {
      new_pose.setX(new_pose.x() + delta_);
    }
    else if (orientation.find("W") != string::npos)
    {
      new_pose.setX(new_pose.x() - delta_);
    }
  }
  else if (action == "left")
  {
    // TOOD: This seems inefficient, but the alternative is annoying because of (-pi,pi] range of orientation
    map<string, string> left_map;
    insert(left_map)("N", "NW")("NE", "N")("E", "NE")("SE", "E")("S", "SE")("SW", "S")("W", "SW")("NW", "W");
    new_pose.setZ(wubble_mdp::convertOrientationString(left_map[wubble_mdp::makeOrientationString(orientation_)]));
  }
  else if (action == "right")
  {
    map<string, string> right_map;
    insert(right_map)("N", "NE")("NE", "E")("E", "SE")("SE", "S")("S", "SW")("SW", "W")("W", "NW")("NW", "N");
    new_pose.setZ(wubble_mdp::convertOrientationString(right_map[wubble_mdp::makeOrientationString(orientation_)]));
  }
  else
  {
    // pose is unchanged
    cerr << action << " is not a valid action!\n";
  }

  //  cout << "NEW " << new_pose.getX() << "," << new_pose.getY() << "," << new_pose.getZ() << endl;

  return new_pose;
}

geometry_msgs::Pose Robot::getPose()
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
