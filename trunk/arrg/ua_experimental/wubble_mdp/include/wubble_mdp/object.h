/*
 * object.h
 *
 *  Created on: Oct 10, 2010
 *      Author: dhewlett
 */

#ifndef OBJECT_H_
#define OBJECT_H_

#include "entity.h"
#include <oomdp_msgs/MDPObjectState.h>

// TODO: There is a lot of overlap with robot, probably should share code
class Object : public Entity
{
public:
  Object(simulator_state::ObjectInfo obj_info);
  Object(oomdp_msgs::MDPObjectState state);
  virtual ~Object();

  virtual oomdp_msgs::MDPObjectState makeObjectState();
  virtual void update(simulator_state::ObjectInfo new_info);

  virtual bool addToWorld();

  virtual std::string getClassString();
  virtual btVector3 getPosition();
  virtual btVector3 getLastPosition();
  virtual geometry_msgs::Pose getPose();

  virtual std::vector<oomdp_msgs::Relation> computePredicates();
  virtual std::vector<oomdp_msgs::Relation> computeBinaryRelations(Entity* other);

  double orientation_;
  double last_x_, last_y_;
  double last_orientation_;

  double size_x_, size_y_, size_z_;
  double mass_;
  bool static_;
  std::string shape_, color_;

  static const double delta_ = 0.5;
};

#endif /* OBJECT_H_ */
