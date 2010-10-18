/*
 * entity.h
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
 */

#ifndef ENTITY_H_
#define ENTITY_H_

#include <iostream>

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>

#include <geometry_msgs/Pose.h>
#include <simulator_state/ObjectInfo.h>
#include <oomdp_msgs/MDPObjectState.h>
#include <oomdp_msgs/Relation.h>

class Entity
{
public:
  virtual oomdp_msgs::MDPObjectState makeObjectState() = 0;
  virtual void update(simulator_state::ObjectInfo new_info) = 0;

  virtual std::string getClassString() = 0;
  virtual btVector3 getPosition() = 0;
  virtual btVector3 getLastPosition() = 0;
  virtual geometry_msgs::Pose getPose() = 0;

  virtual std::vector<oomdp_msgs::Relation> computePredicates() = 0;
  virtual std::vector<oomdp_msgs::Relation> computeBinaryRelations(Entity* other) = 0;

  virtual bool addToWorld() = 0;

  std::string name_;
  double x_, y_;
};

namespace wubble_mdp // These are some useful functions for subclasses, but not really "member" functions
{
std::string makeOrientationString(double yaw);
double extractYaw(simulator_state::ObjectInfo info);
std::string extractOrientationString(simulator_state::ObjectInfo info);
double convertOrientationString(std::string orientation);
std::string doubleToString(double d);
std::map<std::string, std::string> extractAttributeValues(oomdp_msgs::MDPObjectState state);
Entity* makeEntity(oomdp_msgs::MDPObjectState state);
oomdp_msgs::Relation makeRelation(std::string relation, std::vector<std::string> names, bool value);
double deltaAngle(double theta_1, double theta_2);
int chessDistance(btVector3 first_pos, btVector3 second_pos);
double roundToDelta(double number, double delta);
double roundYaw(double yaw);
} // End namespace


#endif /* ENTITY_H_ */
